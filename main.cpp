#include "mbed.h"
#include "PositionSensor.h"
#include "Inverter.h"
#include "SVM.h"
#include "FastMath.h"
#include "Transforms.h"
#include "CurrentRegulator.h"

PositionSensorEncoder encoder(8192,0);
Inverter inverter(PA_5, PB_10, PB_3, PB_7, 0.02014160156, 0.00005);
CurrentRegulator foc(&inverter, &encoder, .005, .5);

Ticker  testing;

using namespace FastMath;
using namespace Transforms;

//float offset = 0;//-0.24;

// Current Sampling IRQ
extern "C" void TIM2_IRQHandler(void) {
  // flash on update event
  if (TIM2->SR & TIM_SR_UIF & TIM2->CNT>0x465) {
      inverter.SampleCurrent();
      }
  TIM2->SR = 0x0; // reset the status register
}

    
void Loop(void){
    foc.Commutate();
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
    //inverter.EnableInverter();

    while(1) {
        
    }
}
