
#include "mbed.h"
#include "SVM.h"
#include "Inverter.h"

SPWM::SPWM(Inverter *inverter, float V_Bus){
    _inverter = inverter;
    _V_Bus = V_Bus;
    }
void SPWM::Update_DTC(float V_Alpha, float V_Beta){
    float DTC_A = V_Alpha/_V_Bus + .5f;
    float DTC_B = 0.5f*(-V_Alpha + 1.73205080757f*V_Beta)/_V_Bus + .5f;
    float DTC_C = 0.5f*(-V_Alpha - 1.73205080757f*V_Beta)/_V_Bus + .5f;
    
    if(DTC_A > .95f) DTC_A = .95f;
    else if(DTC_A < .05f) DTC_A = .05f;
    if(DTC_B > .95f) DTC_B = .95f;
    else if(DTC_B < .05f) DTC_B = .05f;
    if(DTC_C > .95f) DTC_C = .95f;
    else if(DTC_C < .05f) DTC_C = .05f;
    _inverter->SetDTC(DTC_A, DTC_B, DTC_C);
    }