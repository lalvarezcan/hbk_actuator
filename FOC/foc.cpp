
#include "foc.h"
using namespace FastMath;


void abc( float theta, float d, float q, float *a, float *b, float *c){
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    
    *a = cf*d - sf*q;                // Faster Inverse DQ0 transform
    *b = (0.86602540378f*sf-.5f*cf)*d - (-0.86602540378f*cf-.5f*sf)*q;
    *c = (-0.86602540378f*sf-.5f*cf)*d - (0.86602540378f*cf-.5f*sf)*q;
    }
    
    
void dq0(float theta, float a, float b, float c, float *d, float *q){
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    
    float cf = FastCos(theta);
    float sf = FastSin(theta);
    
    *d = 0.6666667f*(cf*a + (0.86602540378f*sf-.5f*cf)*b + (-0.86602540378f*sf-.5f*cf)*c);   ///Faster DQ0 Transform
    *q = 0.6666667f*(-sf*a - (-0.86602540378f*cf-.5f*sf)*b - (0.86602540378f*cf-.5f*sf)*c);
       
    }
    
void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w){
    /// Space Vector Modulation ///
    /// u,v,w amplitude = v_bus for full modulation depth ///
    
    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w))/2.0f;
    *dtc_u = fminf(fmaxf(((u -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w -v_offset)/v_bus + .5f), DTC_MIN), DTC_MAX);  
    
    }

void zero_current(int *offset_1, int *offset_2){                                // Measure zero-offset of the current sensors
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i<n; i++){                                                  // Average n samples of the ADC
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f);                                               // Write duty cycles
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f);
        ADC1->CR2  |= 0x40000000;                                               // Begin sample and conversion
        wait(.001);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC1->DR;
        }
    *offset_1 = adc1_offset/n;
    *offset_2 = adc2_offset/n;
    }

void reset_foc(ControllerStruct *controller){
    TIM1->CCR3 = (PWM_ARR>>1)*(0.5f);
    TIM1->CCR1 = (PWM_ARR>>1)*(0.5f);
    TIM1->CCR2 = (PWM_ARR>>1)*(0.5f);
    controller->i_d_ref = 0;
    controller->i_q_ref = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
    }


void commutate(ControllerStruct *controller, ObserverStruct *observer, GPIOStruct *gpio, float theta){
       /// Observer Prediction ///
       observer->i_d_est += DT*(observer->i_d_dot);
       observer->i_q_est += DT*(observer->i_q_dot);
       
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
       
       float s = FastSin(theta); 
       float c = FastCos(theta);                            
       //dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
       controller->i_d = 0.6666667f*(c*controller->i_a + (0.86602540378f*s-.5f*c)*controller->i_b + (-0.86602540378f*s-.5f*c)*controller->i_c);   ///Faster DQ0 Transform
       controller->i_q = 0.6666667f*(-s*controller->i_a - (-0.86602540378f*c-.5f*s)*controller->i_b - (0.86602540378f*c-.5f*s)*controller->i_c);
        
        controller->i_q_filt = 0.95f*controller->i_q_filt + 0.05f*controller->i_q;
        observer->i_d_m = controller->i_d;
        observer->i_q_m = controller->i_q;
        
        observer->e_d = observer->i_d_m - observer->i_d_est;
        observer->e_q = observer->i_q_m - observer->i_q_est;
        observer->e_d_int += observer->e_d;
        observer->e_q_int += observer->e_q;
        
        observer->i_d_est +=  K_O*observer->e_d + .001f*observer->e_d_int;
        observer->i_q_est += K_O*observer->e_q + .001f*observer->e_q_int;
        
        
        float scog12 = FastSin(12.0f*theta);
        float scog1 = s;
        float cogging_current = 0.25f*scog1 - 0.3f*scog12;
       
       /// PI Controller ///
       float i_d_error = controller->i_d_ref - controller->i_d;
       float i_q_error = controller->i_q_ref - controller->i_q  + cogging_current;
       
       float v_d_ff = 2.0f*(controller->i_d_ref*R_PHASE - controller->dtheta_elec*L_Q*controller->i_q_ref);   //feed-forward voltages
       float v_q_ff =  2.0f*(controller->i_q_ref*R_PHASE  + controller->dtheta_elec*(L_D*controller->i_d_ref + WB));
       
       controller->d_int += i_d_error;   
       controller->q_int += i_q_error;
       
       //v_d_ff = 0;
       //v_q_ff = 0;
       
       limit_norm(&controller->d_int, &controller->q_int, V_BUS/(K_SCALE*I_BW*KI_Q));        // Limit integrators to prevent windup
       controller->v_d = K_SCALE*I_BW*i_d_error + K_SCALE*I_BW*KI_D*controller->d_int;// + v_d_ff;  
       controller->v_q = K_SCALE*I_BW*i_q_error + K_SCALE*I_BW*KI_Q*controller->q_int;// + v_q_ff; 
       
       //controller->v_q = 4.0f;
       //controller->v_d = 0.0f;
       
       //controller->v_d = v_d_ff;
       //controller->v_q = v_q_ff; 
       
       limit_norm(&controller->v_d, &controller->v_q, 1.0f*controller->v_bus);       // Normalize voltage vector to lie within curcle of radius v_bus
       //abc(controller->theta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
    
        controller->v_u = c*controller->v_d - s*controller->v_q;                // Faster Inverse DQ0 transform
        controller->v_v = (0.86602540378f*s-.5f*c)*controller->v_d - (-0.86602540378f*c-.5f*s)*controller->v_q;
        controller->v_w = (-0.86602540378f*s-.5f*c)*controller->v_d - (0.86602540378f*c-.5f*s)*controller->v_q;
       svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w); //space vector modulation

       observer->i_d_dot = 0.5f*(controller->v_d - 2.0f*(observer->i_d_est*R_PHASE - controller->dtheta_elec*L_Q*observer->i_q_est))/L_D;   //feed-forward voltage
       observer->i_q_dot =  0.5f*(controller->v_q - 2.0f*(observer->i_q_est*R_PHASE  + controller->dtheta_elec*(L_D*observer->i_d_est + WB)))/L_Q;
       
       if(PHASE_ORDER){                                                         // Check which phase order to use, 
            TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);                        // Write duty cycles
            TIM1->CCR2 = (PWM_ARR)*(1.0f-controller->dtc_v);
            TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_w);
        }
        else{
            TIM1->CCR3 = (PWM_ARR)*(1.0f-controller->dtc_u);
            TIM1->CCR1 = (PWM_ARR)*(1.0f-controller->dtc_v);
            TIM1->CCR2 =  (PWM_ARR)*(1.0f-controller->dtc_w);
        }

       controller->theta_elec = theta;                                          //For some reason putting this at the front breaks thins
       

       if(controller->loop_count >400){
           //controller->i_q_ref = -controller->i_q_ref;
          controller->loop_count  = 0;
           
           //printf("%.2f  %.2f  %.2f\n\r", controller->i_a, controller->i_b, controller->i_c);
           //printf("%f\n\r", controller->dtheta_mech*GR);
           //pc.printf("%f    %f    %f\n\r", controller->i_a, controller->i_b, controller->i_c);
           //pc.printf("%f    %f\n\r", controller->i_d, controller->i_q);
           //pc.printf("%d    %d\n\r", controller->adc1_raw, controller->adc2_raw);
            }
    }
    
    
void torque_control(ControllerStruct *controller){
    float torque_ref = controller->kp*(controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd*(controller->v_des - controller->dtheta_mech);
    //float torque_ref = -.1*(controller->p_des - controller->theta_mech);
    controller->i_q_ref = torque_ref/KT_OUT;    
    controller->i_d_ref = 0.0f;
    }


/*    
void zero_encoder(ControllerStruct *controller, GPIOStruct *gpio, ){
    
    }
*/    