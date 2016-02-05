
#include "mbed.h"
#include "PositionSensor.h"
#include <math.h>


    
PositionSensorEncoder::PositionSensorEncoder(int CPR, float offset) {
    _CPR = CPR;
    _offset = offset;
    
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
    TIM3->ARR   = 0xffffffff; // reload at 0xfffffff         < TIM auto-reload register
  
    TIM3->CNT = 0x8000;  //reset the counter before we use it  
    
    ZPulse = new InterruptIn(PB_0);
    ZSense = new DigitalIn(PB_0);
    ZPulse->enable_irq();
    ZPulse->rise(this, &PositionSensorEncoder::ZeroEncoderCount);
    ZPulse->mode(PullDown);

    
    //ZTest = new DigitalOut(PC_2);
    //ZTest->write(1);
    
    
}
 
float PositionSensorEncoder::GetMechPosition() {        //returns rotor angle in radians.
    int raw = TIM3->CNT-0x8000;
    if (raw < 0) raw += _CPR;
    if (raw >= _CPR) raw -= _CPR;
    return 6.28318530718f*(raw)/(float)_CPR + _offset;    
}

float PositionSensorEncoder::GetElecPosition() {        //returns rotor electrical angle in radians.
    int raw = TIM3->CNT-0x8000;
    if (raw < 0) raw += _CPR;
    if (raw >= _CPR) raw -= _CPR;
    float signed_elec = fmod((7.0f*(6.28318530718f*(raw)/(float)_CPR + _offset)), 6.28318530718f);
    //float signed_elec = (7*(6.28318530718*(TIM3->CNT-0x8000)/(float)_CPR + _offset));
    if (signed_elec < 0){
        return signed_elec + 6.28318530718f;
        }
    else{
        return signed_elec;
        }
}

void PositionSensorEncoder::ZeroEncoderCount(void){
    if (ZSense->read() == 1){
        if (ZSense->read() == 1){
            TIM3->CNT=0x8000;
            //state = !state;
            //ZTest->write(state);
        }
        }
    }