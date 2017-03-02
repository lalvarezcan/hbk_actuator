#ifndef FOC_H
#define FOC_H

#include "structs.h"


void abc(float theta, float d, float q, float *a, float *b, float *c);
void dq0(float theta, float a, float b, float c, float *d, float *q);
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w);
void zero_current(int *offset_1, int *offset_2);
void reset_foc(ControllerStruct *controller);
#endif
