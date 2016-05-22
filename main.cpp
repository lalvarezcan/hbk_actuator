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
PositionSensorSPI spi(2048, 2.75f, 7);   ///1  I really need an eeprom or something to store this....
//PositionSensorSPI spi(2048, 1.34f, 7); ///2
int motorID = 40; ///1
//int motorID = 50;  ///2

PositionSensorEncoder encoder(1024, 0, 7); 



CurrentRegulator foc(&inverter, &spi, .005, .5);    
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

// HobbyKing-style startup tone.  Just because.
void hk_start(void){
    float dtc  = .1;
    inverter.SetDTC(0, 0, 0);
    inverter.EnableInverter();
    for(int i = 0; i<200; i++){
        //torqueController.SetTorque(.4);
        inverter.SetDTC(dtc, 0, 0);
        wait(0.00047778308);
        //torqueController.SetTorque(-.4);
        inverter.SetDTC(0, dtc, 0);
        wait(0.00047778308);
        }
    for(int i = 0; i<200; i++){
        //torqueController.SetTorque(.4);
        inverter.SetDTC(dtc, 0, 0);
        wait(0.00042565508);
        //torqueController.SetTorque(-.4);
        inverter.SetDTC(0, dtc, 0);
        wait(0.00042565508);
        }    
    for(int i = 0; i<200; i++){
        //torqueController.SetTorque(.4);
        inverter.SetDTC(dtc, 0, 0);
        wait(0.00037921593);
        //torqueController.SetTorque(-.4);
        inverter.SetDTC(0, dtc, 0);
        wait(0.00037921593);
        }   
        inverter.SetDTC(0, 0, 0);
    wait(1);
    for (int j = 0; j<3; j++){
        for(int i = 0; i<240; i++){
            //torqueController.SetTorque(.4);
            inverter.SetDTC(dtc, 0, 0);
            wait(0.00047778308);
            //torqueController.SetTorque(-.4);
            inverter.SetDTC(0, dtc, 0);
            wait(0.00047778308);
            }   
            torqueController.SetTorque(0);
            inverter.SetDTC(0, 0, 0);
            wait(.2);

        }
    
    }


/*      //sinusoidal voltage-mode control, for debugging.
void voltage_foc(void){
    float theta = encoder.GetElecPosition();
    InvPark(v_d, v_q, theta, &v_alpha, &v_beta);
    InvClarke(v_alpha, v_beta, &v_a, &v_b, &v_c);
    svpwm.Update_DTC(v_a, v_b, v_c);
    //output.write(theta/6.28318530718f);
    }
*/

// For decoding serial commands.
 void serialInterrupt(void){
     //wait(.001);
     int i = 0;
     while(pc.readable()){
         buff[i] = pc.getc();
         wait(.0001);
         i++;
         
         }
     int val = (buff[4]<<8) + buff[5];
     int checksum = buff[2]^buff[3]^buff[4]^buff[5];
     int validStart = (buff[0] == 255 && buff[1] == 255 && buff[2]==motorID && checksum==buff[6]);

     if(validStart){

                switch(buff[3]){
                    case 10:
                        cmd_float[1] = (float)val*val_max[1]/65278.0f;
                        break;
                    case 20:
                        cmd_float[2] = (float)val*val_max[2]/65278.0f;
                        break;
                    case 30:
                        cmd_float[0] = (float)val*val_max[0]/65278.0f;
                        break;
                        }
                        }
                        
    
    //pc.printf("%d  %d  %d  %d  %d  %d  %d \n", start1, start2, id, cmd, byte1, byte2, byte3);
    //pc.printf("%f, %f, %f\n", cmd_float[0], cmd_float[1], cmd_float[2]);
    //pc.printf("%d\n", cmd); 
    //pc.printf("%d, %d, %d, %d, %d, %d, %d, %d\n", buff[0], buff[1], buff[2], buff[3], buff[4], buff[5], buff[6], buff[7]);
     }

void Loop(void){
    
    impedanceController.SetImpedance(cmd_float[1], cmd_float[2], cmd_float[0]);
    //impedanceController.SetImpedance(-.04, 0, 0);
    //torqueController.SetTorque(0);
    //foc.Commutate();
    //voltage_foc();

    }

void PrintStuff(void){
    //float v = encoder.GetMechVelocity();
    //float position = encoder.GetElecPosition();
    //float position = encoder.GetMechPosition();
    //float m = spi.GetMechPosition();
    //float e = spi.GetElecPosition();
    //printf("%f\n\r", e);
    //printf("%f   %f   %f   %f \n\r", m, cmd_float[0], cmd_float[1], cmd_float[2]);
    //printf("%d   %d   %d\n\r", raw[0], raw[1], raw[2]);
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
    inverter.SetDTC(0.2, 0.2, 0.2);
    inverter.EnableInverter();
    //hk_start();
    foc.Reset();
    testing.attach(&Loop, .0001);
    NVIC_SetPriority(TIM5_IRQn, 2);
    pc.baud(115200);
    wait(.1);
    while(1) {

    }
}
