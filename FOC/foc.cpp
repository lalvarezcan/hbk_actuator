#include "user_config.h"
#include "foc.h"

//#include "FastMath.h"
//using namespace FastMath;


void abc( float theta, float d, float q, float *a, float *b, float *c){
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    *a = d*cosf(-theta) + q*sinf(-theta);
    *b = d*cosf((2.0f*PI/3.0f)-theta) + q*sinf((2.0f*PI/3.0f)-theta);
    *c =  d*cosf((-2.0f*PI/3.0f)-theta) + q*sinf((-2.0f*PI/3.0f)-theta);
    }
    
void dq0(float theta, float a, float b, float c, float *d, float *q){
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    
    *d = (2.0f/3.0f)*(a*cosf(-theta) + b*cosf((2.0f*PI/3.0f)-theta) + c*cosf((-2.0f*PI/3.0f)-theta));
    *q = (2.0f/3.0f)*(a*sinf(-theta) + b*sinf((2.0f*PI/3.0f)-theta) + c*sinf((-2.0f*PI/3.0f)-theta));
    }
    
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    /// Space Vector Modulation ///
    /// u,v,w amplitude = v_bus for full modulation depth ///
    
    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))/2.0f;
    *dtc_u = fminf(fmaxf(((u - v_offset)*0.5f/v_bus + ((DTC_MAX-DTC_MIN)/2)), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset)*0.5f/v_bus + ((DTC_MAX-DTC_MIN)/2)), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset)*0.5f/v_bus + ((DTC_MAX-DTC_MIN)/2)), DTC_MIN), DTC_MAX);
    
    }

void zero_current(int *offset_1, int *offset_2){                                // Measure zero-offset of the current sensors
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i<n; i++){                                                  // Average n samples of the ADC
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


void commutate(ControllerStruct *controller, GPIOStruct *gpio, float theta){
       /// Commutation Loop ///
       controller->loop_count ++;   
       if(PHASE_ORDER){                                                                          // Check current sensor ordering
           controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    // Calculate phase currents from ADC readings
           controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
           }
        else{
            controller->i_b = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);    
           controller->i_c = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);
           }
       controller->i_a = -controller->i_b - controller->i_c;                                   
       dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
       
       ///Cogging compensation lookup, doesn't actually work yet///
       //int ind = theta * (128.0f/(2.0f*PI));
       //float cogging_current = controller->cogging[ind];
       //float cogging_current = 1.0f*cos(6*theta);
       
       /// PI Controller ///
       float i_d_error = controller->i_d_ref - controller->i_d;
       float i_q_error = controller->i_q_ref - controller->i_q;// + cogging_current;
       float v_d_ff = 2.0f*(2*controller->i_d_ref*R_PHASE);   //feed-forward voltage
       float v_q_ff = 2.0f*(2*controller->i_q_ref*R_PHASE + controller->dtheta_elec*WB*0.8165f);
       controller->d_int += i_d_error;   
       controller->q_int += i_q_error;
       
       //v_d_ff = 0;
       //v_q_ff = 0;
       
       limit_norm(&controller->d_int, &controller->q_int, V_BUS/(K_Q*KI_Q));        // Limit integrators to prevent windup
       //controller->d_int = fminf(fmaxf(controller->d_int, -D_INT_LIM), D_INT_LIM);
       //controller->q_int = fminf(fmaxf(controller->q_int, -Q_INT_LIM), Q_INT_LIM);
       
       
       controller->v_d = K_SCALE*I_BW*i_d_error + K_SCALE*I_BW*controller->d_int;// + v_d_ff;  
       controller->v_q = K_SCALE*I_BW*i_q_error + K_SCALE*I_BW*controller->q_int;// + v_q_ff; 
       
       //controller->v_d = v_d_ff;
       //controller->v_q = v_q_ff; 
       
       limit_norm(&controller->v_d, &controller->v_q, controller->v_bus);       // Normalize voltage vector to lie within curcle of radius v_bus
       abc(controller->theta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       
       if(PHASE_ORDER){                                                         // Check which phase order to use, 
            TIM1->CCR3 = 0x708*(1.0f-controller->dtc_u);                        // Write duty cycles
            TIM1->CCR2 = 0x708*(1.0f-controller->dtc_v);
            TIM1->CCR1 = 0x708*(1.0f-controller->dtc_w);
        }
        else{
            TIM1->CCR3 = 0x708*(1.0f-controller->dtc_u);
            TIM1->CCR1 = 0x708*(1.0f-controller->dtc_v);
            TIM1->CCR2 = 0x708*(1.0f-controller->dtc_w);
        }

       controller->theta_elec = theta;                                          //For some reason putting this at the front breaks thins
       

       //if(controller->loop_count >400){
           //controller->i_q_ref = -controller->i_q_ref;
          // controller->loop_count  = 0;
           
           //printf("%d   %f\n\r", ind, cogging_current);
           //printf("%f\n\r", controller->theta_elec);
           //pc.printf("%f    %f    %f\n\r", controller->i_a, controller->i_b, controller->i_c);
           //pc.printf("%f    %f\n\r", controller->i_d, controller->i_q);
           //pc.printf("%d    %d\n\r", controller->adc1_raw, controller->adc2_raw);
        //    }
    }
/*    
void zero_encoder(ControllerStruct *controller, GPIOStruct *gpio, ){
    
    }
*/    