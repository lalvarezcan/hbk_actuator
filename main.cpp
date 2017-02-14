const unsigned int BOARDNUM = 0x2;

//const unsigned int a_id =                            

const unsigned int TX_ID = 0x0100;
    
const unsigned int cmd_ID = (BOARDNUM<<8) + 0x7;



#include "CANnucleo.h"
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


CANnucleo::CAN          can(PB_8, PB_9);  // CAN Rx pin name, CAN Tx pin name
CANnucleo::CANMessage   rxMsg;
CANnucleo::CANMessage   txMsg;
int                     ledState;
int                     counter = 0;
int canCmd = 1000;
volatile bool           msgAvailable = false;

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
    
void canLoop(void){
    //printf("%d\n\r", canCmd);
    readCAN();
    //sendCMD(TX_ID, canCmd);
    
    //sendCMD(TX_ID+b_ID, b1);
    //sendCMD(TX_ID+c_ID, c1);
    }
    
int id[3] = {0};
float cmd_float[3] = {0.0f};
int raw[3] = {0};
float val_max[3] = {18.0f, 1.0f, 0.1f};    //max angle in radians, stiffness in N-m/rad, damping in N-m*s/rad
int buff[8];
Serial pc(PA_2, PA_3);

Inverter inverter(PA_10, PA_9, PA_8, PA_11, 0.02014160156, 0.00005);  //hall motor
PositionSensorAM5147 spi(16384, 4.7, 21);   ///1  I really need an eeprom or something to store this....
//PositionSensorSPI spi(2048, 1.34f, 7); ///2


PositionSensorEncoder encoder(4096, 0, 21); 



CurrentRegulator foc(&inverter, &spi, &encoder, 0.000033, .005, .55);    
TorqueController torqueController(.082f, &foc);
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
    float torqueCmd = ((float)(canCmd-1000))/100;
    torqueController.SetTorque(torqueCmd);
    if(count>100){
        canLoop();
        //float e = spi.GetElecPosition();
        //float v = encoder.GetMechVelocity();
        //printf("%f\n\r", torqueCmd);
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
    printf("%f\n\r", e);
    //foc.Commutate();
    //float q = foc.GetQ();
    //printf("position: %d   angle: %f    q current: %f\n\r", position, e, q);
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
    inverter.SetDTC(0.0, 0.0, 0.0);
    inverter.EnableInverter();
    foc.Reset();
    testing.attach(&Loop, .000025);
    //canTick.attach(&canLoop, .01);
    //testing.attach(&PrintStuff, .05);
    NVIC_SetPriority(TIM5_IRQn, 2);
    
    can.frequency(1000000);                     // set bit rate to 1Mbps
    can.attach(&onMsgReceived);                 // attach 'CAN receive-complete' interrupt handler
    can.filter(0x020 << 25, 0xF0000004, CANAny, 0);
    
    pc.baud(921600);
    wait(.1);
    pc.printf("HobbyKing Cheeta v1.1\n\r");
    wait(.1);
    while(1) {

    }
}
