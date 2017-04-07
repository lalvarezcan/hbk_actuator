#ifndef LOOPS_H
#define LOOPS_H

#include "structs.h"
#include "PositionSensor.h"


void pd1(float p_des, float v_des, float kp, float kd);
void pd2(float p_des, float kp, float kd);
void torque(float t);

#endif
