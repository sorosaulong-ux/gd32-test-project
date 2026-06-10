/*!
 *  \file    ml_predict.c
 *  \brief   16帧(4s)窗口 LR — 空车 vs 有人
 *
 *  22维特征 = 4全局均值 + 4全局标准差 + 7bin幅值标准差 + 7bin相位环方差
 *  96.4% 准确率, 100% 召回率
 */

#include "ml_predict.h"
#include "model_weights.h"
#include <math.h>
#include <string.h>

#define RAW_FEAT  (4 + ML_N_BINS * 2)   /* = 18 瞬时特征 */
static float  s_buf[ML_WINDOW_SIZE][RAW_FEAT];
static int    s_buf_idx;
static int    s_buf_fill;
static int    s_last_result;
static float  s_last_prob;

static inline float sigmoid(float z)
{
    if (z < -20.0f) return 0.0f;
    if (z >  20.0f) return 1.0f;
    return 1.0f / (1.0f + expf(-z));
}

/* ─── 每帧提取 18 维瞬时特征 ─── */
static void extract_raw(const uwb_radar_result_t *res, float *f)
{
    int i;
    f[0] = (float)res->fp_power;
    f[1] = (float)res->noise;
    {
        float sum = 0.0f, sum2 = 0.0f;
        for (int j = 0; j < UWB_RADAR_CIR_BINS; j++) {
            float ii = (float)res->cir_i[j];
            float qq = (float)res->cir_q[j];
            float mag = sqrtf(ii * ii + qq * qq);
            sum += mag; sum2 += mag * mag;
        }
        float mean = sum / (float)UWB_RADAR_CIR_BINS;
        f[2] = mean;
        f[3] = sqrtf(sum2 / (float)UWB_RADAR_CIR_BINS - mean * mean);
    }
    for (i = 0; i < ML_N_BINS; i++) {
        int b = ml_bins[i];
        float ii = (float)res->cir_i[b];
        float qq = (float)res->cir_q[b];
        f[4 + i]              = sqrtf(ii * ii + qq * qq);
        f[4 + ML_N_BINS + i]  = atan2f(qq, ii);
    }
}

/* ─── 16 帧窗口 → 22 维统计量 ─── */
static void compute_window(float *wf)
{
    int k, frm;

    /* f[0..3]: 4 全局均值 */
    for (k = 0; k < 4; k++) {
        float sum = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++)
            sum += s_buf[frm][k];
        wf[k] = sum / (float)ML_WINDOW_SIZE;
    }

    /* f[4..7]: 4 全局标准差 */
    for (k = 0; k < 4; k++) {
        float sum = 0.0f, sum2 = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float v = s_buf[frm][k];
            sum += v; sum2 += v * v;
        }
        float avg = sum / (float)ML_WINDOW_SIZE;
        wf[4 + k] = sqrtf(sum2 / (float)ML_WINDOW_SIZE - avg * avg);
    }

    /* f[8..14]: 7 bin 幅值标准差 */
    for (k = 0; k < ML_N_BINS; k++) {
        float sum = 0.0f, sum2 = 0.0f;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float v = s_buf[frm][4 + k];
            sum += v; sum2 += v * v;
        }
        float avg = sum / (float)ML_WINDOW_SIZE;
        wf[8 + k] = sqrtf(sum2 / (float)ML_WINDOW_SIZE - avg * avg);
    }

    /* f[15..21]: 7 bin 相位循环方差 */
    for (k = 0; k < ML_N_BINS; k++) {
        float cos_sum = 0.0f, sin_sum = 0.0f;
        int col = 4 + ML_N_BINS + k;
        for (frm = 0; frm < ML_WINDOW_SIZE; frm++) {
            float ph = s_buf[frm][col];
            cos_sum += cosf(ph);
            sin_sum += sinf(ph);
        }
        float R = sqrtf(cos_sum * cos_sum + sin_sum * sin_sum) / (float)ML_WINDOW_SIZE;
        wf[15 + k] = 1.0f - R;
    }
}

int ml_predict(const uwb_radar_result_t *res, float *prob_out)
{
    float raw[RAW_FEAT], wf[ML_N_FEAT];
    int i;

    extract_raw(res, raw);
    memcpy(s_buf[s_buf_idx], raw, sizeof(raw));
    s_buf_idx++;
    if (s_buf_fill < ML_WINDOW_SIZE) s_buf_fill++;

    if (s_buf_fill < ML_WINDOW_SIZE) {
        if (prob_out) *prob_out = 0.0f;
        return 0;
    }

    if (s_buf_idx >= ML_WINDOW_SIZE) {
        compute_window(wf);
        for (i = 0; i < ML_N_FEAT; i++)
            wf[i] = (wf[i] - ml_mu[i]) / ml_sigma[i];

        float z = ml_b;
        for (i = 0; i < ML_N_FEAT; i++)
            z += ml_W[i] * wf[i];

        s_last_prob = sigmoid(z);
        s_last_result = (s_last_prob > 0.5f) ? 1 : 0;
        s_buf_idx = 0;
    }

    if (prob_out) *prob_out = s_last_prob;
    return s_last_result;
}
