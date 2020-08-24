/* 
	开源的AHRS算法。
	MadgwickAHRS
*/

#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

#include "user_math.h"

typedef struct {
	float yaw;
	float pit;
	float rol;
} AHRS_Eulr_t;

typedef	struct {
	float x;
	float y;
	float z;
} AHRS_Accl_t;
	
typedef	struct {
	float x;
	float y;
	float z;
} AHRS_Gyro_t;

typedef	struct {
	float x;
	float y;
	float z;
} AHRS_Magn_t;

typedef struct {
	float q0;
	float q1;
	float q2;
	float q3;

	float inv_sample_freq;
} AHRS_t;

int8_t AHRS_Init(AHRS_t *ahrs, const AHRS_Magn_t *magn, float sample_freq);
int8_t AHRS_Update(AHRS_t *ahrs, const AHRS_Accl_t *accl, const AHRS_Gyro_t *gyro, const AHRS_Magn_t *magn);
int8_t AHRS_GetEulr(AHRS_Eulr_t *eulr, const AHRS_t *ahrs);

#ifdef __cplusplus
}
#endif
