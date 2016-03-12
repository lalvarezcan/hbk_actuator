#include "mbed.h"
#include "PositionSensor.h"
#include "Inverter.h"
#include "SVM.h"
#include "FastMath.h"
#include "Transforms.h"
#include "CurrentRegulator.h"

PositionSensorEncoder encoder(8192,0);
//Inverter inverter(PA_5, PB_10, PB_3, PB_7, 0.02014160156, 0.00005);
Inverter inverter(PA_8, PA_9, PA_10, PB_7, 0.02014160156, 0.00005);
CurrentRegulator foc(&inverter, &encoder, .005, .5);

Ticker  testing;



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
      }
  TIM1->SR = 0x0; // reset the status register
    //GPIOC->ODR ^= (1 << 4); //Toggle pin for debugging
}

    
void Loop(void){
    foc.Commutate();
    }

void PrintStuff(void){
    float velocity = encoder.GetMechVelocity();
    float position = encoder.GetMechPosition();
    printf("%f, %f;\n\r", position, velocity);
    }
/*
void voltage_foc(void){
    theta = encoder.GetElecPosition() + offset;
    InvPark(v_d, v_q, theta, &v_alpha, &v_beta);
    spwm.Update_DTC(v_alpha, v_beta);
    //output.write(theta/6.28318530718f);
    }
 */

       
int main() {
    wait(1);
    testing.attach(&Loop, .0001);
    //testing.attach(&PrintStuff, .05);
    //inverter.SetDTC(.1, 0, 0);
    //inverter.EnableInverter();

    while(1) {
        //printf("%f\n\r", encoder.GetElecPosition());
        //wait(.1);
    }
}
