#include "mbed.h"
#include "PositionSensor.h"
#include "Inverter.h"
#include "SVM.h"
#include "FastMath.h"
#include "Transforms.h"
#include "CurrentRegulator.h"
//#include "TorqueController.h"
//#include "ImpedanceController.h"
PositionSensorEncoder encoder(8192,0);
//Inverter inverter(PA_5, PB_10, PB_3, PB_7, 0.02014160156, 0.00005);
Inverter inverter(PA_10, PA_9, PA_8, PA_11, 0.01007080078, 0.00005);
CurrentRegulator foc(&inverter, &encoder, .005, .5);
SVPWM  svpwm(&inverter, 2.0f);
Ticker  testing;
//Timer t;

float v_d = 0;
float v_q = .1;
float v_alpha = 0;
float v_beta = 0;
float v_a = 0;
float v_b = 0;
float v_c = 0;

//SPI spi(PB_15, PB_14, PB_13);
//GPIOB->MODER = (1 << 8); // set pin 4 to be general purpose output

//DigitalOut chipselect(PB_12);

using namespace FastMath;
using namespace Transforms;


// Current Sampling IRQ
/*
extern "C" void TIM2_IRQHandler(void) {
  // flash on update event
  if (TIM2->SR & TIM_SR_UIF & TIM2->CNT>0x465) {
      inverter.SampleCurrent();
      }
  TIM2->SR = 0x0; // reset the status register
}
*/
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  // toggle on update event
  if (TIM1->SR & TIM_SR_UIF ) {
      inverter.SampleCurrent();
      //wait(.00002);
      //foc.Commutate();
      }
  TIM1->SR = 0x0; // reset the status register
    //GPIOC->ODR ^= (1 << 4); //Toggle pin for debugging
}

void voltage_foc(void){
    float theta = encoder.GetElecPosition();
    InvPark(v_d, v_q, theta, &v_alpha, &v_beta);
    InvClarke(v_alpha, v_beta, &v_a, &v_b, &v_c);
    svpwm.Update_DTC(v_a, v_b, v_c);
    //output.write(theta/6.28318530718f);
    }
   
void Loop(void){
    foc.Commutate();
    //voltage_foc();
    }

void PrintStuff(void){
    float velocity = encoder.GetMechVelocity();
    float position = encoder.GetMechPosition();
    //printf("%f, %f;\n\r", position, velocity);
    printf("%f   %d   %d\n\r", position, TIM3->CNT-0x8000, -2*((int)((TIM3->CR1)>>4)&1)+1);
    }
    


 
 /*
 void gen_sine(void){
     float f = 1.0f;
     float time = t.read();
     float a = .45f*sin(6.28318530718f*f*time) + .5f;
     float b = .45f*sin(6.28318530718f*f*time + 2.09439510239f) + .5f;
     float c = .45f*sin(6.28318530718f*f*time + 4.18879020479f) + .5f;
     inverter.SetDTC(a, b, c);
     }
*/
       
int main() {
    //t.start();
    wait(1);
    testing.attach(&Loop, .0001);
    NVIC_SetPriority(TIM5_IRQn, 1);
    //testing.attach(&gen_sine, .01);
    //testing.attach(&PrintStuff, .1);
    //inverter.SetDTC(.1, 0, 0);
    inverter.EnableInverter();
    //foc.Commutate();
    while(1) {
        //printf("%f\n\r", encoder.GetElecPosition());
        //wait(.1);
    }
}
