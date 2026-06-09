/*!
 *  \file    ml_predict.c
 *  \brief   逻辑回归推理 — 空车(0) vs 有人(1)
 *
 *  模型: LogisticRegression, 18 维特征, sigmoid 输出
 *  准确率: 99.1%  召回率: 95.3%  零误报
 */

#include "ml_predict.h"
#include "model_weights.h"
#include <math.h>
#include <string.h>

/* ─── 关键 bin 索引 (与 training 脚本一致) ─── */
#define N_SEL_BINS  7
static const int sel_bins[N_SEL_BINS] = {2, 8, 25, 31, 63, 69, 103};

/* ─── sigmoid ─── */
static inline float sigmoid(float z)
{
    if (z < -20.0f) return 0.0f;
    if (z >  20.0f) return 1.0f;
    return 1.0f / (1.0f + expf(-z));
}

int ml_predict(const uwb_radar_result_t *res, float *prob_out)
{
    float f[ML_N_FEAT];
    float z;
    int   i;

    /* ── Step 1: 提取特征 ── */

    /* 全局特征 */
    f[0] = (float)res->fp_power;   /* 首径功率 |I|+|Q| */
    f[1] = (float)res->noise;      /* 噪声均值 */

    /* 幅度均值 & 标准差 — Euclidean: sqrt(I²+Q²) */
    {
        float sum  = 0.0f;
        float sum2 = 0.0f;
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

    /* 逐 bin 特征: 幅度 + 相位 */
    for (i = 0; i < N_SEL_BINS; i++) {
        int b = sel_bins[i];
        float ii = (float)res->cir_i[b];
        float qq = (float)res->cir_q[b];
        f[4 + i * 2]     = sqrtf(ii * ii + qq * qq);   /* mag */
        f[4 + i * 2 + 1] = atan2f(qq, ii);              /* phase */
    }

    /* ── Step 2: 标准化 ── */
    for (i = 0; i < ML_N_FEAT; i++) {
        f[i] = (f[i] - ml_mu[i]) / ml_sigma[i];
    }

    /* ── Step 3: 逻辑回归 z = W·x + b ── */
    z = ml_b;
    for (i = 0; i < ML_N_FEAT; i++) {
        z += ml_W[i] * f[i];
    }

    /* ── Step 4: sigmoid → 概率 ── */
    float prob = sigmoid(z);
    if (prob_out) *prob_out = prob;

    return (prob > 0.5f) ? 1 : 0;
}
