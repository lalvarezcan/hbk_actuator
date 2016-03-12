#ifndef CURRENTRETULATOR_H
#define CURRENTREGULATOR_H
#include "Inverter.h"
#include "SVM.h"
#include "PositionSensor.h"

class CurrentRegulator{
    public:
        CurrentRegulator(Inverter *inverter, PositionSensor *position_sensor, float Kp, float Ki);
        void UpdateRef(float D, float Q);
        void Commutate();
    private:
        float IQ_Ref, ID_Ref, V_Q, V_D, V_Alpha, V_Beta, I_Q, I_D, I_A, I_B, I_C, I_Alpha, I_Beta, theta_elec, _Kp, _Ki;
        float Q_Integral, D_Integral, Q_Error, D_Error, Int_Max, DTC_Max;
        void SampleCurrent();
        void SetVoltage();
        void Update();
        void SendSPI();
        Inverter* _Inverter;
        PositionSensor* _PositionSensor;
        SPWM* PWM;
        //Serial* pc;
        //int count;
            
    
    
    };
    
    
#endif