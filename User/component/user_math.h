/*
  自定义的数学运算。
*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx.h"
#define ARM_MATH_CM4

#include <float.h>
#include <math.h>

#define M_DEG2RAD_MULT (0.01745329251f)
#define M_RAD2DEG_MULT (57.2957795131f)

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_2PI
#define M_2PI 6.28318530717958647692f
#endif

typedef struct {
  float vx;     /* 前后平移 */
  float vy;     /* 左右平移 */
  float wz;     /* 转动 */
} MoveVector_t; /* 移动向量 */

float InvSqrt(float x);

float AbsClip(float in, float limit);

float Sign(float in);

void ResetMoveVector(MoveVector_t *mv);

float CircleError(float sp, float fb, float range);

void CircleAdd(float *origin, float delta, float range);

#ifdef __cplusplus
}
#endif
