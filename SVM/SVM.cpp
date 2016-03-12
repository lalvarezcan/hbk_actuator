
#include "mbed.h"
#include "SVM.h"
#include "Inverter.h"

SPWM::SPWM(Inverter *inverter, float V_Bus){
    _inverter = inverter;
    _V_Bus = V_Bus;
    }

void SPWM::Update_DTC(float V_A, float V_B, float V_C){
    float DTC_A = V_A/_V_Bus + .5f;
    float DTC_B = V_B/_V_Bus + .5f;
    float DTC_C = V_C/_V_Bus + .5f;
    
    if(DTC_A > .95f) DTC_A = .95f;
    else if(DTC_A < .05f) DTC_A = .05f;
    if(DTC_B > .95f) DTC_B = .95f;
    else if(DTC_B < .05f) DTC_B = .05f;
    if(DTC_C > .95f) DTC_C = .95f;
    else if(DTC_C < .05f) DTC_C = .05f;
    _inverter->SetDTC(DTC_A, DTC_B, DTC_C);
    }

SVPWM::SVPWM(Inverter *inverter, float V_Bus){
    _inverter = inverter;
    _V_Bus = V_Bus;
    }
    
void SVPWM::Update_DTC(float V_A, float V_B, float V_C){
    float DTC_A = V_A/_V_Bus + .5f;
    float DTC_B = V_B/_V_Bus + .5f;
    float DTC_C = V_C/_V_Bus + .5f;
    
    if(DTC_A > .95f) DTC_A = .95f;
    else if(DTC_A < .05f) DTC_A = .05f;
    if(DTC_B > .95f) DTC_B = .95f;
    else if(DTC_B < .05f) DTC_B = .05f;
    if(DTC_C > .95f) DTC_C = .95f;
    else if(DTC_C < .05f) DTC_C = .05f;
    _inverter->SetDTC(DTC_A, DTC_B, DTC_C);
    }
    