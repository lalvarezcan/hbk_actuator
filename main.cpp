const unsigned int BOARDNUM = 0x2;

//const unsigned int a_id =                            

const unsigned int TX_ID = 0x0100;
    
const unsigned int cmd_ID = (BOARDNUM<<8) + 0x7;



#include "CANnucleo.h"
#include "mbed.h"
#include "PositionSensor.h"
#include "structs.h"
#include "foc.h"
#include "hw_setup.h"
#include "math_ops.h"
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"

GPIOStruct gpio;
ControllerStruct controller;
COMStruct com;



CANnucleo::CAN          can(PB_8, PB_9);  // CAN Rx pin name, CAN Tx pin name
CANnucleo::CANMessage   rxMsg;
CANnucleo::CANMessage   txMsg;
int                     ledState;
int                     counter = 0;
int canCmd = 1000;
volatile bool           msgAvailable = false;

DigitalOut toggle(PA_0);
Ticker  loop;
/**
 * @brief   'CAN receive-complete' interrup handler.
 * @note    Called on arrival of new CAN message.
 *          Keep it as short as possible.
 * @param   
 * @retval  
 */
void onMsgReceived() {
    msgAvailable = true;
    //printf("ping\n\r");
}

void sendCMD(int TX_addr, int val){
    txMsg.clear();      //clear Tx message storage
    txMsg.id = TX_addr;
    txMsg << val;
    can.write(txMsg);
    //wait(.1);
    
    }
    
void readCAN(void){
    if(msgAvailable) { 
    msgAvailable = false;               // reset flag for next use
    can.read(rxMsg);                    // read message into Rx message storage
    // Filtering performed by software:           
    if(rxMsg.id == cmd_ID) {             // See comments in CAN.cpp for filtering performed by hardware
            rxMsg >> canCmd;  
            }             // extract first data item
        }
        }
    
void cancontroller(void){
    //printf("%d\n\r", canCmd);
    readCAN();
    //sendCMD(TX_ID, canCmd);
    
    //sendCMD(TX_ID+b_ID, b1);
    //sendCMD(TX_ID+c_ID, c1);
    }
    

Serial pc(PA_2, PA_3);

PositionSensorAM5147 spi(16384, 4.7, NPP);   ///1  I really need an eeprom or something to store this....
//PositionSensorEncoder encoder(4096, 0, 21); 

int count = 0;
void commutate(void){
       
       count ++;

       //pc.printf("%f\n\r", controller.theta_elec);
        //Get rotor angle
       //spi.GetMechPosition();
       controller.i_b = I_SCALE*(float)(controller.adc2_raw - controller.adc2_offset);    //Calculate phase currents from ADC readings
       controller.i_c = I_SCALE*(float)(controller.adc1_raw - controller.adc1_offset);
       controller.i_a = -controller.i_b - controller.i_c;
       
       
       dq0(controller.theta_elec, controller.i_a, controller.i_b, controller.i_c, &controller.i_d, &controller.i_q);    //dq0 transform on currents
       
       ///Control Law///
       float i_d_error = controller.i_d_ref - controller.i_d;
       float i_q_error = controller.i_q_ref - controller.i_q;
       float v_d_ff = controller.i_d_ref*R_TOTAL;   //feed-forward voltage
       float v_q_ff = controller.i_q_ref*R_TOTAL;
       controller.d_int += i_d_error;   
       controller.q_int += i_q_error;
       
       limit_norm(&controller.d_int, &controller.q_int, V_BUS/(K_Q*KI_Q));
       //controller.d_int = fminf(fmaxf(controller.d_int, -D_INT_LIM), D_INT_LIM);
       //controller.q_int = fminf(fmaxf(controller.q_int, -Q_INT_LIM), Q_INT_LIM);
       
       
       controller.v_d = K_D*i_d_error + K_D*KI_D*controller.d_int;// + v_d_ff;  
       controller.v_q = K_Q*i_q_error + K_Q*KI_Q*controller.q_int;// + v_q_ff; 
       //controller.v_d = 10*v_d_ff;
       //controller.v_q = 10*v_q_ff; 
       limit_norm(&controller.v_d, &controller.v_q, controller.v_bus);
       
       abc(controller.theta_elec, controller.v_d, controller.v_q, &controller.v_u, &controller.v_v, &controller.v_w); //inverse dq0 transform on voltages
       svm(controller.v_bus, controller.v_u, controller.v_v, controller.v_w, &controller.dtc_u, &controller.dtc_v, &controller.dtc_w); //space vector modulation

       gpio.pwm_u->write(1.0f-controller.dtc_u);  //write duty cycles
       gpio.pwm_v->write(1.0f-controller.dtc_v);
       gpio.pwm_w->write(1.0f-controller.dtc_w);  
       
       controller.theta_elec = spi.GetElecPosition();   
       //TIM1->CCR1 = (int)(controller.dtc_u * 0x8CA);//gpio.pwm_u->write(1.0f-controller.dtc_u);  //write duty cycles
       //TIM1->CCR2 = (int)(controller.dtc_v * 0x8CA);//gpio.pwm_v->write(1.0f-controller.dtc_v);
       //TIM1->CCR3 = (int)(controller.dtc_w * 0x8CA);//gpio.pwm_w->write(1.0f-controller.dtc_w);

       //gpio.pwm_u->write(1.0f - .1f);  //write duty cycles
       //gpio.pwm_v->write(1.0f - .1f);
       //gpio.pwm_w->write(1.0f - .15f);


       if(count >1000){
           controller.i_q_ref = -controller.i_q_ref;
           count  = 0;
           //pc.printf("%f\n\r", controller.theta_elec);
           //pc.printf("%f    %f    %f\n\r", controller.i_a, controller.i_b, controller.i_c);
           //pc.printf("%f    %f\n\r", controller.i_d, controller.i_q);
           //pc.printf("%d    %d\n\r", controller.adc1_raw, controller.adc2_raw);
            }
       }
       

// Current Sampling IRQ
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {
        //toggle = 1;
        ADC1->CR2  |= 0x40000000;  
        //volatile int delay;
        //for (delay = 0; delay < 55; delay++);
        
        controller.adc2_raw = ADC2->DR;
        controller.adc1_raw = ADC1->DR;
        //toggle = 0;
        commutate();        
         
      }
  TIM1->SR = 0x0; // reset the status register
}

       
       
int main() {

    controller.v_bus = V_BUS;
    spi.ZeroPosition();
    Init_All_HW(&gpio);

    wait(.1);
    //TIM1->CR1 |= TIM_CR1_UDIS;
    gpio.enable->write(1);
    gpio.pwm_u->write(1.0f);  //write duty cycles
    gpio.pwm_v->write(1.0f);
    gpio.pwm_w->write(1.0f);
    zero_current(&controller.adc1_offset, &controller.adc2_offset);
    reset_foc(&controller);
    TIM1->CR1 ^= TIM_CR1_UDIS; //enable interrupt
    gpio.enable->write(1);
       //gpio.pwm_u->write(1.0f - .05f);  //write duty cycles
       //gpio.pwm_v->write(1.0f - .05f);
       //gpio.pwm_w->write(1.0f - .1f);
    
    wait(.1);
    NVIC_SetPriority(TIM5_IRQn, 2);
    //loop.attach(&commutate, .000025);
    can.frequency(1000000);                     // set bit rate to 1Mbps
    can.attach(&onMsgReceived);                 // attach 'CAN receive-complete' interrupt handler
    can.filter(0x020 << 25, 0xF0000004, CANAny, 0);
    
    pc.baud(921600);
    wait(.01);
    pc.printf("HobbyKing Cheetah v1.1\n\r");
    pc.printf("ADC1 Offset: %d    ADC2 Offset: %d", controller.adc1_offset, controller.adc2_offset);
    wait(.01);
    

       controller.i_d_ref = 0;
       controller.i_q_ref = 0;
    while(1) {

    }
}
