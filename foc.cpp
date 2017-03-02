
#include "foc.h"
#include "mbed.h"
#include "hw_config.h"
#include "math.h"
#include "math_ops.h"
//#include "FastMath.h"
//using namespace FastMath;


void abc( float theta, float d, float q, float *a, float *b, float *c){
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    *a = d*cosf(-theta) + q*sinf(-theta);
    *b = d*cosf((2.0f*PI/3.0f)-theta) + q*sinf((2.0f*PI/3.0f)-theta);
    *c =  d*cosf((-2.0f*PI/3.0f)-theta) + q*sinf((-2.0f*PI/3.0f)-theta);
    }
    
void dq0(float theta, float a, float b, float c, float *d, float *q){
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    
    *d = (2.0f/3.0f)*(a*cosf(-theta) + b*cosf((2.0f*PI/3.0f)-theta) + c*cosf((-2.0f*PI/3.0f)-theta));
    *q = (2.0f/3.0f)*(a*sinf(-theta) + b*sinf((2.0f*PI/3.0f)-theta) + c*sinf((-2.0f*PI/3.0f)-theta));
    }
    
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    ///u,v,w amplitude = v_bus for full modulation depth///
    
    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))/2.0f;
    *dtc_u = fminf(fmaxf(((u - v_offset)*0.5f/v_bus + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset)*0.5f/v_bus + 0.5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset)*0.5f/v_bus + 0.5f), DTC_MIN), DTC_MAX);
    
    }

void zero_current(int *offset_1, int *offset_2){
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i<n; i++){
        ADC1->CR2  |= 0x40000000; 
        wait(.001);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC1->DR;
        }
    *offset_1 = adc1_offset/n;
    *offset_2 = adc2_offset/n;
    }

void reset_foc(ControllerStruct *controller){
    controller->q_int = 0;
    controller->d_int = 0;
    }
