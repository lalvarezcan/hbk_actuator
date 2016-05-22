
#include "mbed.h"
#include "PositionSensor.h"
//#include <math.h>

PositionSensorSPI::PositionSensorSPI(int CPR, float offset, int ppairs){
    //_CPR = CPR;
    _CPR = CPR;
    _ppairs = ppairs;
    _offset = offset;
    rotations = 0;
    spi = new SPI(PC_12, PC_11, PC_10);
    spi->format(16, 0);
    cs = new DigitalOut(PA_15);
    cs->write(1);
    MechOffset = 0;
    }

int PositionSensorSPI::GetRawPosition(){
        cs->write(0);
    int response = spi->write(0)>>4;
    cs->write(1);
    return response;
    }
    

float PositionSensorSPI::GetMechPosition(){
    cs->write(0);
    int response = spi->write(0)>>4;
    cs->write(1);
    if(response - old_counts > _CPR/4){
        rotations -= 1;
        }
    else if (response - old_counts < -_CPR/4){
        rotations += 1;
        }
    old_counts = response;
    MechPosition = (6.28318530718f * ((float) response+(_CPR*rotations)))/ (float)_CPR;
    //return MechPosition - MechOffset;
    return MechPosition;
    }

float PositionSensorSPI::GetElecPosition(){
    cs->write(0);
    int response = spi->write(0)>>4;
    cs->write(1);
    float elec = ((6.28318530718f/(float)_CPR) * (float) ((_ppairs*response)%_CPR)) - _offset;
    if(elec < 0) elec += 6.28318530718f;
    return elec;
    }

float PositionSensorSPI::GetMechVelocity(){
    return 0;
    }

void PositionSensorSPI::ZeroPosition(){
    rotations = 0;
    MechOffset = GetMechPosition();
    }
    

    
PositionSensorEncoder::PositionSensorEncoder(int CPR, float offset, int ppairs) {
    _ppairs = ppairs;
    _CPR = CPR;
    _offset = offset;
    MechPosition = 0;
    
    // Enable clock for GPIOA
    __GPIOA_CLK_ENABLE(); //equivalent from hal_rcc.h
 
    GPIOA->MODER   |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 ;           //PA6 & PA7 as Alternate Function   /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->OTYPER  |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 ;                 //PA6 & PA7 as Inputs               /*!< GPIO port output type register,        Address offset: 0x04      */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 ;     //Low speed                         /*!< GPIO port output speed register,       Address offset: 0x08      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_1 ;           //Pull Down                         /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x22000000 ;                                          //AF02 for PA6 & PA7                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //nibbles here refer to gpio8..15   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   
    // configure TIM3 as Encoder input
    // Enable clock for TIM3
    __TIM3_CLK_ENABLE();
 
    TIM3->CR1   = 0x0001;     // CEN(Counter ENable)='1'     < TIM control register 1
    TIM3->SMCR  = TIM_ENCODERMODE_TI12;     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM3->CCMR1 = 0xf1f1;     // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1, maximum digital filtering
    TIM3->CCMR2 = 0x0000;     //                             < TIM capture/compare mode register 2
    TIM3->CCER  = 0x0011;     // CC1P CC2P                   < TIM capture/compare enable register
    TIM3->PSC   = 0x0000;     // Prescaler = (0+1)           < TIM prescaler
    TIM3->ARR   = CPR-1; // reload at 0xfffffff         < TIM auto-reload register
  
    TIM3->CNT = 0x000;  //reset the counter before we use it  
    
    // Extra Timer for velocity measurement
    
    __TIM2_CLK_ENABLE();
    TIM3->CR2 = 0x030;  //MMS = 101
    
    TIM2->PSC = 0x03;
    //TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->SMCR = 0x24; //TS = 010 for ITR2, SMS = 100 (reset counter at edge)
    TIM2->CCMR1 = 0x3;// CC1S = 11, IC1 mapped on TRC
    
    //TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->CCER |= TIM_CCER_CC1P;
    //TIM2->CCER |= TIM_CCER_CC1NP;
    TIM2->CCER |= TIM_CCER_CC1E;
    
    
    TIM2->CR1 = 0x01;       //CEN
    
    TIM3->CR1   = 0x01;     // CEN
    ZPulse = new InterruptIn(PC_4);
    ZSense = new DigitalIn(PC_4);
    //ZPulse = new InterruptIn(PB_0);
    //ZSense = new DigitalIn(PB_0);
    ZPulse->enable_irq();
    ZPulse->rise(this, &PositionSensorEncoder::ZeroEncoderCount);
    //ZPulse->fall(this, &PositionSensorEncoder::ZeroEncoderCountDown);
    ZPulse->mode(PullDown);
    flag = 0;

    
    //ZTest = new DigitalOut(PC_2);
    //ZTest->write(1);
    
    
}
 
float PositionSensorEncoder::GetMechPosition() {        //returns rotor angle in radians.
    int raw = TIM3->CNT;
    float unsigned_mech = (6.28318530718f/(float)_CPR) * (float) ((raw)%_CPR);
    return (float) unsigned_mech;// + 6.28318530718f* (float) rotations;
}

float PositionSensorEncoder::GetElecPosition() {        //returns rotor electrical angle in radians.
    int raw = TIM3->CNT;
    float elec = ((6.28318530718f/(float)_CPR) * (float) ((_ppairs*raw)%_CPR)) - _offset;
    if(elec < 0) elec += 6.28318530718f;
    return elec;
}


    
float PositionSensorEncoder::GetMechVelocity(){
    float out = 0;
    float rawPeriod = TIM2->CCR1; //Clock Ticks
    
    float  dir = -2.0f*(float)(((TIM3->CR1)>>4)&1)+1.0f;    // +/- 1
    float meas = dir*90000000.0f*(6.28318530718f/(float)_CPR)/rawPeriod; 
    out = meas;
    if(meas == vel_old){
        out = .95f*out_old;
        }
    else{
        out = meas;
        }
    
    vel_old = meas;
    out_old = out;
    return out;
    }
    
float PositionSensorEncoder::GetElecVelocity(){
    return _ppairs*GetMechVelocity();
    }
    
void PositionSensorEncoder::ZeroEncoderCount(void){
    if (ZSense->read() == 1 & flag == 0){
        if (ZSense->read() == 1){
            GPIOC->ODR ^= (1 << 4);   
            TIM3->CNT = 0x000;
            //state = !state;
            //ZTest->write(state);
            GPIOC->ODR ^= (1 << 4);
            //flag = 1;
        }
        }
    }
    
void PositionSensorEncoder::ZeroEncoderCountDown(void){
    if (ZSense->read() == 0){
        if (ZSense->read() == 0){
            GPIOC->ODR ^= (1 << 4);
            flag = 0;
            float dir = -2.0f*(float)(((TIM3->CR1)>>4)&1)+1.0f;
            if(dir != dir){
                dir = dir;
                rotations +=  dir;
                }

            GPIOC->ODR ^= (1 << 4);

        }
        }
    }