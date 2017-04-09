/// high-bandwidth 3-phase motor control, for robots
/// Written by benkatz, with much inspiration from bayleyw, nkirkby, scolton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com

/// Written for the STM32F446, but can be implemented on other STM32 MCU's with some further register-diddling

#define REST_MODE 0
#define CALIBRATION_MODE 1
#define TORQUE_MODE 2
#define PD_MODE 3
#define SETUP_MODE 4
#define ENCODER_MODE 5


const unsigned int BOARDNUM = 0x2;
//const unsigned int a_id =                            
const unsigned int TX_ID = 0x0100;
const unsigned int cmd_ID = (BOARDNUM<<8) + 0x7;

float __float_reg[64];
int __int_reg[256];

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
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include "user_config.h"
#include "PreferenceWriter.h"

PreferenceWriter prefs(6);

GPIOStruct gpio;
ControllerStruct controller;
COMStruct com;
VelocityEstimatorStruct velocity;


CANnucleo::CAN          can(PB_8, PB_9);                                        // CAN Rx pin name, CAN Tx pin name
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
    msgAvailable = false;                                                       // reset flag for next use
    can.read(rxMsg);                                                            // read message into Rx message storage
    // Filtering performed by software:           
    if(rxMsg.id == cmd_ID) {                                                    // See comments in CAN.cpp for filtering performed by hardware
            rxMsg >> canCmd;                                                    // extract first data item
            }                                                                   
        }
    }
    
void cancontroller(void){
    //printf("%d\n\r", canCmd);
    readCAN();
    //sendCMD(TX_ID, canCmd);

    }
    

Serial pc(PA_2, PA_3);

PositionSensorAM5147 spi(16384, 0.0, NPP);   
PositionSensorEncoder encoder(4096, 0, 21); 

volatile int count = 0;
volatile int state = REST_MODE;
volatile int state_change;

void enter_menu_state(void){
    printf("\n\r\n\r\n\r");
    printf(" Commands:\n\r");
    printf(" t - Torque Mode\n\r");
    printf(" p - PD Mode\n\r");
    printf(" c - Calibrate Encoder\n\r");
    printf(" s - Setup\n\r");
    printf(" e - Display Encoder\n\r");
    printf(" esc - Exit to Menu\n\r");
    state_change = 0;
    gpio.enable->write(0);
    }

void enter_setup_state(void){
    printf("\n\r\n\r Configuration Options \n\r\n\n");
    printf(" %-7s %-25s %-5s %-5s %-5s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
    printf(" %-7s %-25s %-5s %-5s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    printf(" %-7s %-25s %-5s %-5s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
    printf(" %-7s %-25s %-5s %-5s %.1f\n\r", "l", "Torque Limit (N-m)", "0.0", "18.0", TORQUE_LIMIT);
    printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
    state_change = 0;
    }
    
void enter_torque_mode(void){
    controller.i_d_ref = 0;
    controller.i_q_ref = 1;                                                     // Current Setpoints
    reset_foc(&controller);                                                     // Tesets integrators, and other control loop parameters
    gpio.enable->write(1);                                                      // Enable gate drive
    GPIOC->ODR ^= (1 << 5);                                                     // Turn on status LED
    state_change = 0;
    }
    
void calibrate(void){
    gpio.enable->write(1);                                                      // Enable gate drive
    GPIOC->ODR ^= (1 << 5);                                                     // Turn on status LED
    order_phases(&spi, &gpio, &controller, &prefs);                             // Check phase ordering
    calibrate(&spi, &gpio, &controller, &prefs);                                // Perform calibration procedure
    GPIOC->ODR ^= (1 << 5);                                                     // Turn off status LED
    wait(.2);
    gpio.enable->write(0);                                                      // Turn off gate drive
    printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
     state_change = 0;
     
    }
    
void print_encoder(void){
    spi.Sample();
    wait(.001);
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", spi.GetMechPosition(), spi.GetElecPosition(), spi.GetRawPosition());
    wait(.05);
    }

/// Current Sampling Interrupt ///
/// This runs at 40 kHz, regardless of of the mode the controller is in ///
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {
        //toggle = 1;

        ///Sample current always ///
        ADC1->CR2  |= 0x40000000;                                               // Begin sample and conversion
        //volatile int delay;   
        //for (delay = 0; delay < 55; delay++);
        controller.adc2_raw = ADC2->DR;                                         // Read ADC1 and ADC2 Data Registers
        controller.adc1_raw = ADC1->DR;
        ///
        
        /// Check state machine state, and run the appropriate function ///
        //printf("%d\n\r", state);
        switch(state){
            case REST_MODE:                                                     // Do nothing until
                if(state_change){
                    enter_menu_state();
                    }
                break;
            
            case CALIBRATION_MODE:                                              // Run encoder calibration procedure
                if(state_change){
                    calibrate();
                    }
                break;
             
            case TORQUE_MODE:                                                   // Run torque control
                if(state_change){
                    enter_torque_mode();
                    }
                count++;
                controller.theta_elec = spi.GetElecPosition();                  
                commutate(&controller, &gpio, controller.theta_elec);           // Run current loop
                spi.Sample();                                                   // Sample position sensor
                if(count > 100){
                     count = 0;
                     //readCAN();
                     //controller.i_q_ref = ((float)(canCmd-1000))/100;
                    //pc.printf("%f\n\r ", controller.theta_elec);
                     }
                break;
            
            case PD_MODE:
                break;
            case SETUP_MODE:
                if(state_change){
                    enter_setup_state();
                }
                break;
            case ENCODER_MODE:
                print_encoder();
                break;
                }
                
   
            
      }
  TIM1->SR = 0x0;                                                               // reset the status register
}


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void){
    while(pc.readable()){
        char c = pc.getc();
        if(c == 27){
                state = REST_MODE;
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
        if(state == REST_MODE){
            switch (c){
                case 'c':
                    state = CALIBRATION_MODE;
                    state_change = 1;
                    break;
                case 't':
                    state = TORQUE_MODE;
                    state_change = 1;
                    break;
                case 'e':
                    state = ENCODER_MODE;
                    state_change = 1;
                    break;
                case 's':
                    state = SETUP_MODE;
                    state_change = 1;
                    break;
                    }
                }
        else if(state == SETUP_MODE){
            if(c == 13){
                switch (cmd_id){
                    case 'b':
                        I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                        break;
                    case 'i':
                        CAN_ID = atoi(cmd_val);
                        break;
                    case 'l':
                        TORQUE_LIMIT = fmaxf(fminf(atof(cmd_val), 18.0f), 0.0f);
                        break;
                    default:
                        printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
                        break;
                    }
                    
                if (!prefs.ready()) prefs.open();
                prefs.flush();                                                  // Write new prefs to flash
                prefs.close();    
                prefs.load();                                              
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
            else{
                if(char_count == 0){cmd_id = c;}
                else{
                    cmd_val[char_count-1] = c;
                    
                }
                pc.putc(c);
                char_count++;
                }
            }
        else if (state == ENCODER_MODE){
            switch (c){
                case 27:
                    state = REST_MODE;
                    state_change = 1;
                    break;
                    }
            }
            
        }
    }
       
int main() {
    
    controller.v_bus = V_BUS;
    controller.mode = 0;
    Init_All_HW(&gpio);                                                         // Setup PWM, ADC, GPIO

    wait(.1);
    //TIM1->CR1 |= TIM_CR1_UDIS;
    gpio.enable->write(1);                                                      // Enable gate drive
    gpio.pwm_u->write(1.0f);                                                    // Write duty cycles
    gpio.pwm_v->write(1.0f);
    gpio.pwm_w->write(1.0f);
    zero_current(&controller.adc1_offset, &controller.adc2_offset);             // Measure current sensor zero-offset
    //gpio.enable->write(0);
    reset_foc(&controller);                                                     // Reset current controller
    
    TIM1->CR1 ^= TIM_CR1_UDIS; //enable interrupt
    
    wait(.1);
    NVIC_SetPriority(TIM5_IRQn, 2);                                             // set interrupt priority

    can.frequency(1000000);                                                     // set bit rate to 1Mbps
    can.attach(&onMsgReceived);                                                 // attach 'CAN receive-complete' interrupt handler
    can.filter(0x020 << 25, 0xF0000004, CANAny, 0);
    
    
    prefs.load();                                                               // Read flash
    spi.SetElecOffset(E_OFFSET);                                                // Set position sensor offset
    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));
    spi.WriteLUT(lut);                                                          // Set potision sensor nonlinearity lookup table
    
    pc.baud(115200);                                                            // set serial baud rate
    wait(.01);
    pc.printf("\n\r\n\r HobbyKing Cheetah\n\r\n\r");
    wait(.01);
    printf("\n\r Debug Info:\n\r");
    printf(" ADC1 Offset: %d    ADC2 Offset: %d\n\r", controller.adc1_offset, controller.adc2_offset);
    printf(" Position Sensor Electrical Offset:   %.4f\n\r", E_OFFSET);
    printf(" CAN ID:  %d\n\r", CAN_ID);
        
    pc.attach(&serial_interrupt);                                               // attach serial interrupt
    
    state_change = 1;

    
    while(1) {

    }
}
