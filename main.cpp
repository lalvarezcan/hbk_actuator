/// high-bandwidth 3-phase motor control, for robots
/// Written by benkatz, with much inspiration from bayleyw, nkirkby, scolton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/// Written for the STM32F446, but can be implemented on other STM32 MCU's with some further register-diddling



const unsigned int BOARDNUM = 0x2;

//const unsigned int a_id =                            

const unsigned int TX_ID = 0x0100;
    
const unsigned int cmd_ID = (BOARDNUM<<8) + 0x7;



#include "CANnucleo.h"
#include "mbed.h"
#include "PositionSensor.h"
#include "structs.h"
#include "foc.h"
#include "calibration.h"
#include "hw_setup.h"
#include "math_ops.h"
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"

GPIOStruct gpio;
ControllerStruct controller;
COMStruct com;
VelocityEstimatorStruct velocity;



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

PositionSensorAM5147 spi(16384, -0.4658, NPP);   ///1  I really need an eeprom or something to store this....
PositionSensorEncoder encoder(4096, 0, 21); 

int count = 0;
int mode = 0;

    
float velocity_estimate(void){
    velocity.vel_2 = encoder.GetMechVelocity();
    velocity.vel_1 = spi.GetMechVelocity();
    
    velocity.ts = .01f;
    velocity.vel_1_est = velocity.ts*velocity.vel_1 + (1-velocity.ts)*velocity.vel_1_est;   //LPF
    velocity.vel_2_est = (1-velocity.ts)*(velocity.vel_2_est + velocity.vel_2 - velocity.vel_2_old);    //HPF
    velocity.est = velocity.vel_1_est + velocity.vel_2_est;
    
    velocity.vel_1_old = velocity.vel_1;
    velocity.vel_2_old = velocity.vel_2;
    return velocity.est;
    }

// Current Sampling Interrupt
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {
        //toggle = 1;
        count++;
        ADC1->CR2  |= 0x40000000;                                       //Begin sample and conversion
        //volatile int delay;   
        //for (delay = 0; delay < 55; delay++);
        
        if(controller.mode == 2){
            controller.adc2_raw = ADC2->DR;
            controller.adc1_raw = ADC1->DR;
            
            //toggle = 0;
            
            spi.Sample();
            controller.theta_elec = spi.GetElecPosition();
            commutate(&controller, &gpio, controller.theta_elec);        
            }
         
         
         //controller.dtheta_mech = spi.GetMechVelocity();
         //controller.dtheta_elec = encoder.GetElecVelocity();
         //ontroller.dtheta_mech = encoder.GetMechVelocity();
         //controller.i_q_ref = 2.0f*controller.dtheta_mech;
         
         
         //float v1 = encoder.GetMechVelocity();
         //float v2 = spi.GetMechVelocity();
         
         
         if(count > 100){
             count = 0;
             //for (int i = 1; i<16; i++){
                //pc.printf("%d\n\r ", spi.GetRawPosition());
             //   }
                //pc.printf("\n\r");
                //pc.printf("%.4f %.4f  %.4f\n\r",velocity.vel_1 ,velocity.vel_2, velocity.est );

             }
         
            
      }
  TIM1->SR = 0x0; // reset the status register
}


void enter_torque_mode(void){
    controller.mode = 2;
    TIM1->CR1 ^= TIM_CR1_UDIS;                                          //enable interrupts
    controller.i_d_ref = 0;
    controller.i_q_ref = 1;
    reset_foc(&controller);                                             //resets integrators, and other control loop parameters
    gpio.enable->write(1);
    GPIOC->ODR ^= (1 << 5);                                             //turn on LED
    }
    
void enter_calibration_mode(void){
    controller.mode = 1;
    TIM1->CR1 ^= TIM_CR1_UDIS;
    gpio.enable->write(1);
    GPIOC->ODR ^= (1 << 5);     
    //calibrate_encoder(&spi);
    order_phases(&spi, &gpio);
    calibrate(&spi, &gpio);
    GPIOC->ODR ^= (1 << 5);
    wait(.2);
    gpio.enable->write(0);
     TIM1->CR1 ^= TIM_CR1_UDIS;
     controller.mode = 0;
    }
    
       
int main() {

    controller.v_bus = V_BUS;
    controller.mode = 0;
    //spi.ZeroPosition(); 
    Init_All_HW(&gpio);                                                 // Setup PWM, ADC, GPIO

    wait(.1);
    //TIM1->CR1 |= TIM_CR1_UDIS;
    gpio.enable->write(1);
    gpio.pwm_u->write(1.0f);                                            //write duty cycles
    gpio.pwm_v->write(1.0f);
    gpio.pwm_w->write(1.0f);
    zero_current(&controller.adc1_offset, &controller.adc2_offset);     //Measure current sensor zero-offset
    //gpio.enable->write(0);
    reset_foc(&controller);
    
    //TIM1->CR1 ^= TIM_CR1_UDIS; //enable interrupt
    //gpio.enable->write(1);
    //gpio.pwm_u->write(1.0f - .05f);  //write duty cycles
    //gpio.pwm_v->write(1.0f - .05f);
    //gpio.pwm_w->write(1.0f - .1f);
    
    wait(.1);
    NVIC_SetPriority(TIM5_IRQn, 2);                                     // set interrupt priority

    can.frequency(1000000);                                             // set bit rate to 1Mbps
    can.attach(&onMsgReceived);                                         // attach 'CAN receive-complete' interrupt handler
    can.filter(0x020 << 25, 0xF0000004, CANAny, 0);
    
    pc.baud(115200);
    wait(.01);
    pc.printf("HobbyKing Cheetah v1.1\n\r");
    pc.printf("ADC1 Offset: %d    ADC2 Offset: %d", controller.adc1_offset, controller.adc2_offset);
    wait(.01);
    
    
    enter_calibration_mode();
    enter_torque_mode();

    
    while(1) {

    }
}
