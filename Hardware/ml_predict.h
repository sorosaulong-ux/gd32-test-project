/*!
 *  \file    ml_predict.h
 *  \brief   逻辑回归推理 — 空车 vs 有人
 *
 *  特征(18维): fp_power, noise, mag_mean, mag_std, bin[2,8,25,31,63,69,103]×{mag,phase}
 */

#ifndef ML_PREDICT_H_
#define ML_PREDICT_H_

#include "uwb.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

int ml_predict(const uwb_radar_result_t *res, float *prob_out);

#ifdef __cplusplus
}
#endif

#endif
