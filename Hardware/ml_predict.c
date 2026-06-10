/*!
 *  \file    ml_predict.c
 *  \brief   16帧(4s)窗口 LR — 空车 vs 有人
 *
 *  每帧入环形缓冲 → 满16帧后计算25维窗口统计量
 *  → 标准化 → sigmoid → 返回预测（保持到下一窗口）
 *
 *  90.0% 准确率, 93.5% 召回率, 3% 误报率
 */

#include "ml_predict.h"
#include "model_weights.h"
#include <math.h>
#include <string.h>

/* ─── 环形缓冲: 16帧 × 18维瞬时特征 ─── */
#define RAW_FEAT  (4 + ML_N_BINS * 2)   /* = 18 */
static float  s_buf[ML_WINDOW_SIZE][RAW_FEAT];
static int    s_buf_idx;
static int    s_buf_fill;

/* ─── 上一窗口的预测结果（帧间保持） ─── */
static int    s_last_result;
static float  s_last_prob;

/* ─── sigmoid ─── */
static inline float sigmoid(float z)
{
    if (z < -20.0f) return 0.0f;
    if (z >  20.0f) return 1.0f;
    return 1.0f / (1.0f + expf(-z));
}

/* ====================================================================
 *  extract_raw — 每帧提取 18 维瞬时特征
 * ====================================================================*/
static void extract_raw(const uwb_radar_result_t *res, float *f)
{
    int i;

    f[0] = (float)res->fp_power;
    f[1] = (float)res->noise;

    /* mag_mean, mag_std over all bins */
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

    /* 逐 bin: 幅度 + 相位 */
    for (i = 0; i < ML_N_BINS; i++) {
        int b = ml_bins[i];
        float ii = (float)res->cir_i[b];
        float qq = (float)res->cir_q[b];
        f[4 + i]       = sqrtf(ii * ii + qq * qq);
        f[4 + ML_N_BINS + i] = atan2f(qq, ii);
    }
}

/* ====================================================================
 *  compute_window — 25 维窗口统计量 (NF = 4 + 3×ML_N_BINS)
 *
 *  f[0..3]:   mean of {fp_power, noise, mag_mean, mag_std}
 *  f[4..10]:  mean of 7 bin magnitudes
 *  f[11..17]: std  of 7 bin magnitudes
 *  f[18..24]: circular variance of 7 bin phases
 * ====================================================================*/
static void compute_window(float *wf)
{
    int k, frm;

    /* ── 前 4 维: 窗口均值 ── */
    for (k = 0; k < 4; k++) {
        float sum = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++)
            sum += s_buf[frm][k];
        wf[k] = sum / (float)ML_WINDOW_SIZE;
    }

    /* ── 7 bin 幅度的均值 ── */
    for (k = 0; k < ML_N_BINS; k++) {
        float sum = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++)
            sum += s_buf[frm][4 + k];
        wf[4 + k] = sum / (float)ML_WINDOW_SIZE;
    }

    /* ── 7 bin 幅度的标准差 ── */
    for (k = 0; k < ML_N_BINS; k++) {
        float sum = 0.0f, sum2 = 0.0f;
        int col = 4 + k;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float v = s_buf[frm][col];
            sum  += v;
            sum2 += v * v;
        }
        float avg = sum / (float)ML_WINDOW_SIZE;
        wf[11 + k] = sqrtf(sum2 / (float)ML_WINDOW_SIZE - avg * avg);
    }

    /* ── 7 bin 相位循环方差 ── */
    for (k = 0; k < ML_N_BINS; k++) {
        float cos_sum = 0.0f, sin_sum = 0.0f;
        int col = 4 + ML_N_BINS + k;  /* phase column */
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float ph = s_buf[frm][col];
            cos_sum += cosf(ph);
            sin_sum += sinf(ph);
        }
        float R = sqrtf(cos_sum * cos_sum + sin_sum * sin_sum) / (float)ML_WINDOW_SIZE;
        wf[18 + k] = 1.0f - R;
    }
}

/* ====================================================================
 *  ml_predict — 每帧调用
 *
 *  前 ML_WINDOW_SIZE 帧: 填充窗口, 返回 s_last_result (0)
 *  满窗口后: 每帧返回上次计算结果, 直到下一窗口边界
 *  窗口边界帧 (每 16 帧): 计算 + 返回新结果
 * ====================================================================*/
int ml_predict(const uwb_radar_result_t *res, float *prob_out)
{
    float raw[RAW_FEAT];
    float wf[ML_N_FEAT];
    int   i;

    /* ── 1. 入缓冲 ── */
    extract_raw(res, raw);
    memcpy(s_buf[s_buf_idx], raw, sizeof(raw));
    s_buf_idx++;
    if (s_buf_fill < ML_WINDOW_SIZE) s_buf_fill++;

    /* ── 2. 未满 → 返回旧结果 ── */
    if (s_buf_fill < ML_WINDOW_SIZE) {
        if (prob_out) *prob_out = 0.0f;
        return 0;
    }

    /* ── 3. 满, 且 s_buf_idx 回到 0 (窗口边界) → 计算 ── */
    if (s_buf_idx >= ML_WINDOW_SIZE) {
        /* 排好序: 按时间顺填写 wf */
        compute_window(wf);

        /* 标准化 */
        for (i = 0; i < ML_N_FEAT; i++)
            wf[i] = (wf[i] - ml_mu[i]) / ml_sigma[i];

        /* z = W·x + b */
        float z = ml_b;
        for (i = 0; i < ML_N_FEAT; i++)
            z += ml_W[i] * wf[i];

        s_last_prob = sigmoid(z);
        s_last_result = (s_last_prob > 0.5f) ? 1 : 0;
        s_buf_idx = 0;  /* reset for next window */
    }

    if (prob_out) *prob_out = s_last_prob;
    return s_last_result;
}
