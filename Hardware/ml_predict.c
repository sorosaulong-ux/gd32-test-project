/*!
 *  \file    ml_predict.c
 *  \brief   滑动窗口逻辑回归 — 8帧窗口 → 29维特征 → sigmoid
 *
 *  93.5% 准确率, 90.7% 召回率 (3数据集训练)
 */

#include "ml_predict.h"
#include "model_weights.h"
#include <math.h>
#include <string.h>

/* ─── 环形缓冲区 (8帧 × 18原始特征) ─── */
#define RAW_FEAT   18
static float  s_buf[ML_WINDOW_SIZE][RAW_FEAT];
static int    s_buf_idx;
static int    s_buf_fill;               /* 已填充帧数 */

/* ─── sigmoid ─── */
static inline float sigmoid(float z)
{
    if (z < -20.0f) return 0.0f;
    if (z >  20.0f) return 1.0f;
    return 1.0f / (1.0f + expf(-z));
}

/* ====================================================================
 *  extract_raw — 从 CIR 结果提取 18 维瞬时特征
 * ====================================================================*/
static void extract_raw(const uwb_radar_result_t *res, float *f)
{
    int i;

    /* 全局 */
    f[0] = (float)res->fp_power;
    f[1] = (float)res->noise;

    {
        float sum = 0.0f, sum2 = 0.0f;
        for (int j = 0; j < UWB_RADAR_CIR_BINS; j++) {
            float ii = (float)res->cir_i[j];
            float qq = (float)res->cir_q[j];
            float mag = sqrtf(ii * ii + qq * qq);
            sum  += mag;
            sum2 += mag * mag;
        }
        float mean = sum / (float)UWB_RADAR_CIR_BINS;
        f[2] = mean;
        f[3] = sqrtf(sum2 / (float)UWB_RADAR_CIR_BINS - mean * mean);
    }

    /* 关键 bin: 幅度 + 相位 */
    for (i = 0; i < ML_N_BINS; i++) {
        int b = ml_bins[i];
        float ii = (float)res->cir_i[b];
        float qq = (float)res->cir_q[b];
        f[4 + i]  = sqrtf(ii * ii + qq * qq);      /* mag */
        f[11 + i] = atan2f(qq, ii);                 /* phase */
    }
}

/* ====================================================================
 *  compute_window — 从环形缓冲计算 29 维窗口统计量
 *
 *  f[0..7]:   mean/std  of {fp_power, noise, mag_mean, mag_std}
 *  f[8..21]:  mean/std  of 7 bin magnitudes
 *  f[22..28]: circular variance of 7 bin phases
 * ====================================================================*/
static void compute_window(float *wf)
{
    int k, frm;

    /* ── 全局 4 特征 × (mean, std) = 8 维 ── */
    for (k = 0; k < 4; k++) {
        float sum = 0.0f, sum2 = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float v = s_buf[frm][k];
            sum  += v;
            sum2 += v * v;
        }
        float avg = sum / (float)ML_WINDOW_SIZE;
        wf[k * 2]     = avg;                         /* mean */
        wf[k * 2 + 1] = sqrtf(sum2 / (float)ML_WINDOW_SIZE - avg * avg);  /* std */
    }

    /* ── 7 bin 幅度 × (mean, std) = 14 维 ── */
    for (k = 0; k < ML_N_BINS; k++) {
        float sum = 0.0f, sum2 = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float v = s_buf[frm][4 + k];
            sum  += v;
            sum2 += v * v;
        }
        float avg = sum / (float)ML_WINDOW_SIZE;
        wf[8 + k * 2]     = avg;                     /* mean */
        wf[8 + k * 2 + 1] = sqrtf(sum2 / (float)ML_WINDOW_SIZE - avg * avg);  /* std */
    }

    /* ── 7 bin 相位 循环方差 = 7 维 ── */
    for (k = 0; k < ML_N_BINS; k++) {
        float cos_sum = 0.0f, sin_sum = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float ph = s_buf[frm][11 + k];
            cos_sum += cosf(ph);
            sin_sum += sinf(ph);
        }
        float R = sqrtf(cos_sum * cos_sum + sin_sum * sin_sum) / (float)ML_WINDOW_SIZE;
        wf[22 + k] = 1.0f - R;                       /* circular variance */
    }
}

/* ====================================================================
 *  ml_predict — public API
 *
 *  每帧调用，返回 0=空车 / 1=有人。
 *  前 ML_WINDOW_SIZE 帧用于填充窗口（返回 0, 不预测）。
 *  prob_out 可 NULL。
 * ====================================================================*/
int ml_predict(const uwb_radar_result_t *res, float *prob_out)
{
    float raw[RAW_FEAT];
    float wf[ML_N_FEAT];
    float z;
    int   i;

    /* ── 1. 提取瞬时特征, 入环形缓冲 ── */
    extract_raw(res, raw);
    memcpy(s_buf[s_buf_idx], raw, sizeof(raw));
    s_buf_idx = (s_buf_idx + 1) % ML_WINDOW_SIZE;
    if (s_buf_fill < ML_WINDOW_SIZE) s_buf_fill++;
    if (s_buf_fill < ML_WINDOW_SIZE) {
        if (prob_out) *prob_out = 0.0f;
        return 0;  /* 窗口未满, 不预测 */
    }

    /* ── 2. 计算窗口统计量 ── */
    compute_window(wf);

    /* ── 3. 标准化 ── */
    for (i = 0; i < ML_N_FEAT; i++) {
        wf[i] = (wf[i] - ml_mu[i]) / ml_sigma[i];
    }

    /* ── 4. 逻辑回归 z = W·x + b ── */
    z = ml_b;
    for (i = 0; i < ML_N_FEAT; i++) {
        z += ml_W[i] * wf[i];
    }

    /* ── 5. sigmoid ≈ 概率 ── */
    float prob = sigmoid(z);
    if (prob_out) *prob_out = prob;

    return (prob > 0.5f) ? 1 : 0;
}
