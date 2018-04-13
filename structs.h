#ifndef STRUCTS_H
#define STRUCTS_H

//#include "CANnucleo.h"
#include "mbed.h"
#include "FastPWM.h"


typedef struct{
    DigitalOut *enable;
    DigitalOut *led;
    FastPWM *pwm_u, *pwm_v, *pwm_w;
    } GPIOStruct;
    
typedef struct{
    
    }COMStruct;
    
typedef struct{
    int adc1_raw, adc2_raw, adc3_raw;
    float i_a, i_b, i_c;
    float v_bus;
    float theta_mech, theta_elec;
    float dtheta_mech, dtheta_elec, dtheta_elec_filt;
    float i_d, i_q, i_q_filt;
    float v_d, v_q;
    float dtc_u, dtc_v, dtc_w;
    float v_u, v_v, v_w;
    float d_int, q_int;
    int adc1_offset, adc2_offset;
    float i_d_ref, i_q_ref;
    int loop_count;
    int timeout;
    int mode;
    int ovp_flag;
    float p_des, v_des, kp, kd, t_ff;
    float cogging[128];
    } ControllerStruct;

typedef struct{
    float theta_m, theta_est;
    float thetadot_m, thetadot_est;
    float i_d_m, i_d_est;
    float i_q_m, i_q_est;
    float i_d_dot, i_q_dot;
    float e_d, e_q;
    float e_d_int, e_q_int;
    } ObserverStruct;
    
#endif
