#include "mbed.h"
#include "PositionSensor.h"
#include "Inverter.h"
#include "SVM.h"
#include "FastMath.h"
#include "Transforms.h"
#include "CurrentRegulator.h"
#include "TorqueController.h"
#include "ImpedanceController.h"

///SPI Input Stuff
//DigitalIn cselect(PB_12);
//InterruptIn select(PB_12);
//DigitalIn mosi(PB_15);
//SPISlave input(PB_15, PB_14, PB_13, PB_12);

int id[3] = {0};
float cmd_float[3] = {0.0f};
int raw[3] = {0};
float val_max[3] = {18.0f, 1.0f, 0.1f};
int buff[8];
Serial pc(PA_2, PA_3);

//PositionSensorEncoder encoder(8192,4.0f);
//Inverter inverter(PA_5, PB_10, PB_3, PB_7, 0.02014160156, 0.00005);
Inverter inverter(PA_10, PA_9, PA_8, PA_11, 0.02014160156, 0.00005);  //hall motter
//Inverter inverter(PA_10, PA_9, PA_8, PB_7, 0.01007080078, 0.00005);  //test motter
PositionSensorSPI spi(2048, 2.75f);   ///1  I really need an eeprom or something to store this....
//PositionSensorSPI spi(2048, 1.34f); ///2
//PositionSensorSPI spi(2048, 1);
int motorID = 40; ///1
//int motorID = 50;  ///2

PositionSensorEncoder encoder(1024, 0);

CurrentRegulator foc(&inverter, &spi, .005, .5);  //hall sensor
TorqueController torqueController(.031f, &foc);
ImpedanceController impedanceController(&torqueController, &spi, &encoder);

//CurrentRegulator foc(&inverter, &encoder, .005, .5);    //test motter
//SVPWM  svpwm(&inverter, 2.0f);

Ticker  testing;
//Timer t;

/*
float v_d = 0;
float v_q = .1;
float v_alpha = 0;
float v_beta = 0;
float v_a = 0;
float v_b = 0;
float v_c = 0;
*/
float ref = 0.0;
int count = 0;

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
      //foc.Commutate(); ///Putting the loop here doesn't work for some reason.  Need to figure out why
      }
  TIM1->SR = 0x0; // reset the status register
    //GPIOC->ODR ^= (1 << 4); //Toggle pin for debugging
}



/*
void voltage_foc(void){
    float theta = encoder.GetElecPosition();
    InvPark(v_d, v_q, theta, &v_alpha, &v_beta);
    InvClarke(v_alpha, v_beta, &v_a, &v_b, &v_c);
    svpwm.Update_DTC(v_a, v_b, v_c);
    //output.write(theta/6.28318530718f);
    }
*/
/*
void read(void){
    int startByte;
    if(input.receive()){
        //startByte = input.read();
        //if(startByte == 65535){
            //startByte = input.read();
            //wait(.000005);
            raw[0] = input.read();
            raw[1] = input.read();
            raw[2] = input.read();
            id[0] = raw[0]>>14;
            id[1] = raw[1]>>14;
            id[2] = raw[2]>>14;
            printf("%d   %d   %d\n\r", raw[0], raw[1], raw[2]);
            for(int i = 0; i<3; i++){
                    cmd_float[id[i]] = (val_max[id[i]])*(float)(raw[i] - (id[i]<<14))/16383.0f;
                    }
           //  }
         // else{
          //    input.read();
          //    input.read();
           //   input.read();
           //   }
        //printf("%d   %d   %d \n\r", raw[0], raw[1], raw[2]);
        }
        
    }
    
*/

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
    //impedanceController.SetImpedance(-.4, -0.006, 0);

    count = count+1;
    
    if(count > 1000){
        //ref= -1*ref;
        //printf("%f   %f   %f \n\r",  cmd_float[0], cmd_float[1], cmd_float[2]);
        //float e = spi.GetElecPosition();
        //printf("%f\n\r", e);
        count = 0;
        }
    
     
    //torqueController.SetTorque(0);
    //foc.Commutate();
    //voltage_foc();
    }

void PrintStuff(void){
    //float v = encoder.GetMechVelocity();
    //float position = encoder.GetElecPosition();
    //float position = encoder.GetMechPosition();
    //float m = spi.GetMechPosition();
    float e = spi.GetElecPosition();
    printf("%f\n\r", e);
    //printf("%f   %f   %f   %f \n\r", m, cmd_float[0], cmd_float[1], cmd_float[2]);
    //printf("%d   %d   %d\n\r", raw[0], raw[1], raw[2]);
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
    //mosi.mode(PullDown);
    //cselect.mode(PullUp);
    inverter.DisableInverter();
    spi.ZeroPosition();
    //input.format(16, 0);
    //input.frequency(100000);
    //select.fall(&read);


    //NVIC_SetPriority(EXTI15_10_IRQn, 1);
    wait(.1);
    inverter.SetDTC(0.2, 0.2, 0.2);
    inverter.EnableInverter();
    foc.Reset();
    testing.attach(&Loop, .0001);
    NVIC_SetPriority(TIM5_IRQn, 2);
    pc.baud(115200);
    //pc.attach(&serialInterrupt);
    //printf("hello\n\r");
    //testing.attach(&gen_sine, .01);
    //testing.attach(&PrintStuff, .1);
    //inverter.SetDTC(.05, 0, 0);
    //inverter.DisableInverter();
    //foc.Commutate();
    wait(.5);
    while(1) {
        //printf("%f\n\r", encoder.GetElecPosition());
        //wait(.1);
    }
}
