#ifndef STRUCTS_H
#define STRUCTS_H

//#include "CANnucleo.h"
#include "mbed.h"
#include "FastPWM.h"


typedef struct{
    DigitalOut *enable;
    FastPWM *pwm_u, *pwm_v, *pwm_w;
    } GPIOStruct;
    
typedef struct{
    
    }COMStruct;
    
typedef struct{
    int adc1_raw, adc2_raw;
    float i_a, i_b, i_c;
    float v_bus;
    float theta_mech, theta_elec;
    float dtheta_mech, dtheta_elec;
    float i_d, i_q;
    float v_d, v_q;
    float dtc_u, dtc_v, dtc_w;
    float v_u, v_v, v_w;
    float d_int, q_int;
    int adc1_offset, adc2_offset;
    float i_d_ref, i_q_ref;
    } ControllerStruct;


#endif
