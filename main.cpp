#include "mbed.h"
#include "PositionSensor.h"
#include "Inverter.h"
#include "SVM.h"
#include "FastMath.h"
#include "Transforms.h"
#include "CurrentRegulator.h"
#include "TorqueController.h"
#include "ImpedanceController.h"

using namespace FastMath;
using namespace Transforms;

int id[3] = {0};
float cmd_float[3] = {0.0f};
int raw[3] = {0};
float val_max[3] = {18.0f, 1.0f, 0.1f};    //max angle in radians, stiffness in N-m/rad, damping in N-m*s/rad
int buff[8];
Serial pc(PA_2, PA_3);

Inverter inverter(PA_10, PA_9, PA_8, PA_11, 0.02014160156, 0.00005);  //hall motor
PositionSensorAM5147 spi(16384, 1.65f, 21);   ///1  I really need an eeprom or something to store this....
//PositionSensorSPI spi(2048, 1.34f, 7); ///2


PositionSensorEncoder encoder(4096, 0, 21); 



CurrentRegulator foc(&inverter, &spi, &encoder, 0.000033, .005, .55);    
TorqueController torqueController(.031f, &foc);
ImpedanceController impedanceController(&torqueController, &spi, &encoder);

Ticker  testing;




// Current Sampling IRQ
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {
      inverter.SampleCurrent();
      //foc.Commutate(); ///Putting the loop here doesn't work for some reason.  Need to figure out why
      }
  TIM1->SR = 0x0; // reset the status register
}

int count = 0;
void Loop(void){
    count++;
    //impedanceController.SetImpedance(cmd_float[1], cmd_float[2], cmd_float[0]);
    //impedanceController.SetImpedance(.1, -0.01, 0);
    
    torqueController.SetTorque(.1);
    //foc.Commutate();
    //voltage_foc();
    if(count>2000){
        //float e = spi.GetElecPosition();
        //float v = encoder.GetMechVelocity();
        //printf("%f\n\r", v);
        //printf("IA: %f   IB: %f  IC: %f\n\r", inverter.I_A, inverter.I_B, inverter.I_C);
        count = 0;
        }

    }

void PrintStuff(void){
    //inverter.SetDTC(0.03, 0.0, 0.0);

    //float v = encoder.GetMechVelocity();
    //float position = encoder.GetElecPosition();
    int position = spi.GetRawPosition();
    //float m = spi.GetMechPosition();
    float e = spi.GetElecPosition();
    foc.Commutate();
    float q = foc.GetQ();
    printf("position: %d   angle: %f    q current: %f\n\r", position, e, q);
    //inverter.getCurrent()
    //printf("%f   %f   %f   %f \n\r", m, cmd_float[0], cmd_float[1], cmd_float[2]);
    //printf("%d   %d   %d\n\r", raw[0], raw[1], raw[2]);
    //printf("IA: %f   IB: %f  IC: %f\n\r", inverter.I_A, inverter.I_B, inverter.I_C);
    }

 /*
 ////Throw some sines on the phases.  useful to make sure the hardware works.
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
    inverter.DisableInverter();
    spi.ZeroPosition();
    wait(.1);
    inverter.SetDTC(0.03, 0.0, 0.0);
    inverter.EnableInverter();
    foc.Reset();
    testing.attach(&Loop, .000025);
    //testing.attach(&PrintStuff, .05);
    NVIC_SetPriority(TIM5_IRQn, 2);
    pc.baud(921600);
    pc.printf("HobbyKing Cheeta v1.1\n\r");
    wait(.1);
    while(1) {

    }
}
