#ifndef MATH_OPS_H
#define MATH_OPS_H

#define PI 3.14159265359f

#include "math.h"

float fmaxf(float x, float y);
float fminf(float x, float y);
float fmaxf3(float x, float y, float z);
float fminf3(float x, float y, float z);
void limit_norm(float *x, float *y, float limit);

#endif
