#ifndef INVERTER_H
#define INVERTER_H
#include "FastPWM.h"
#include "mbed.h"


class Inverter{
    public:
        Inverter(PinName PinA, PinName PinB, PinName PinC, PinName PinEnable, float I_Scale, float Period);
        void SetDTC(float DTC_A, float DTC_B, float DTC_C);
        void EnableInverter();
        void DisableInverter();
        void InitCurrentSense();
        void SampleCurrent(); 
        void GetCurrent(float *A, float *B, float *C);
        float I_A, I_B, I_C, _I_Scale;

    private:
        float I_B_Offset, I_C_Offset;
        void ZeroCurrent(void);
        FastPWM *PWM_A, *PWM_B, *PWM_C;  
        DigitalOut *Enable, *Dbg; 
        AnalogIn *Current_B, *Current_C;
    };

#endif