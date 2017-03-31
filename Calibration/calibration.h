#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "foc.h"
#include "mbed.h"
#include "PositionSensor.h"



void order_phases(PositionSensor *ps, GPIOStruct *gpio);
void calibrate(PositionSensor *ps, GPIOStruct *gpio);
#endif
