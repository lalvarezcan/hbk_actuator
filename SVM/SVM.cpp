
#include "mbed.h"
#include "SVM.h"
#include "Inverter.h"
#define min(x,y,z) (x < y ? (x < z ? x : z) : (y < z ? y : z))
#define max(x,y,z) (x > y ? (x > z ? x : z) : (y > z ? y : z))

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
    
    float Voff = (min(V_A, V_B, V_C) + max(V_A, V_B, V_C))/2.0f;
    
    V_A = V_A - Voff;
    V_B = V_B - Voff;
    V_C = V_C - Voff;
    
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
    