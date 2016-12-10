#include "Transforms.h"
#include "CurrentRegulator.h"
#include "Inverter.h"
#include "SVM.h"
#include "Transforms.h"
#include "PositionSensor.h"

using namespace Transforms;

CurrentRegulator::CurrentRegulator(Inverter *inverter, PositionSensor *position_sensor, PositionSensor *velocity_sensor, float L, float Kp, float Ki){
    _Inverter = inverter;
    PWM = new SVPWM(inverter, 2.0);
    _PositionSensor = position_sensor;
    _VelocitySensor = velocity_sensor;
    IQ_Ref = 0;
    Q_Max = 40.0f;
    ID_Ref = 0;
    V_Q = 0;
    V_D = 0;
    V_Alpha = 0;
    V_Beta = 0;
    V_A = 0;
    V_B = 0;
    V_C = 0;
    I_Q = 0;
    I_D = 0;
    I_A = 0;
    I_B = 0;
    I_C = 0;
    I_Alpha = 0;
    I_Beta = 0;
    IQ_Old = 0;
    ID_Old = 0;
    count = 0;
    _L = L;
    _Kp = Kp;
    _Ki = Ki;
    Q_Integral = 0;
    D_Integral = 0;
    Int_Max = .9;
    DTC_Max = .97;
    //theta_elec = _PositionSensor->GetElecPosition();

    
    }

void CurrentRegulator::SendSPI(){
    
    
    }
    
float CurrentRegulator::GetQ(){
    return I_Q;
    }
    
void CurrentRegulator::Reset(void){
    IQ_Ref = 0;
    ID_Ref = 0;
    V_Q = 0;
    V_D = 0;
    V_Alpha = 0;
    V_Beta = 0;
    V_A = 0;
    V_B = 0;
    V_C = 0;
    I_Q = 0;
    I_D = 0;
    I_A = 0;
    I_B = 0;
    I_C = 0;
    I_Alpha = 0;
    I_Beta = 0;
    //pc->printf("%f, %f\n\r", Q_Integral, D_Integral);
    wait(.03);
    Q_Integral = 0;
    D_Integral = 0;    
    }
    
void CurrentRegulator::UpdateRef(float D, float Q){
    if(Q>Q_Max){
        Q = Q_Max;
        }
    else if(Q<-Q_Max){
        Q = -Q_Max;
        }
    if(D>Q_Max){
        D = Q_Max;
        }
    else if(D<-Q_Max){
        D = -Q_Max;
        }
    IQ_Ref = Q;
    ID_Ref = D;
    }

void CurrentRegulator::SampleCurrent(){
    _Inverter->GetCurrent(&I_A, &I_B, &I_C);
    Clarke(I_A, I_B, &I_Alpha, &I_Beta);
    float ID_Sample, IQ_Sample;
    //Park(I_Alpha, I_Beta, theta_elec, &I_D, &I_Q);
    
    Park(I_Alpha, I_Beta, theta_elec, &ID_Sample, &IQ_Sample);    
    I_D = 1.0f*ID_Sample + 0.0f*ID_Old;
    I_Q = 1.0f*IQ_Sample + 0.0f*IQ_Old;
    ID_Old = I_D;
    IQ_Old = I_Q;
    //count += 1;
    //if(count > 10000) {
    //    count=0;
    //    printf("I_A:  %f        I_C:  %f       I_C:  %f\n\r", I_A, I_B, I_C);
    //IQ_Ref = -IQ_Ref;
    //    }

    //DAC->DHR12R1 = (int) (I_Q*490.648f) + 2048;
    //DAC->DHR12R1 = (int) (I_Alpha*4096.0f) + 2048;
    }
    
void CurrentRegulator::Update(){
        float Q_Error = IQ_Ref - I_Q;
        float D_Error = ID_Ref - I_D;
        float w_elec = _VelocitySensor->GetElecVelocity();
        
        Q_Integral += Q_Error*_Ki*_Kp;
        D_Integral += D_Error*_Ki*_Kp;
        
        if (Q_Integral > Int_Max) Q_Integral = Int_Max;
        else if(Q_Integral < -Int_Max) Q_Integral = -Int_Max;
        if (D_Integral > Int_Max) D_Integral = Int_Max;
        else if(D_Integral < -Int_Max) D_Integral = -Int_Max;       
         
        V_Q = Q_Integral + _Kp*Q_Error;
        //V_Q = V_Q - w_elec*I_D;
        V_D = D_Integral + _Kp*D_Error;     
        //V_D = V_D + w_elec*I_Q;   //decoupling needs moar testing
    }
        
void CurrentRegulator::SetVoltage(){
    InvPark(V_D, V_Q, theta_elec, &V_Alpha, &V_Beta);
    InvClarke(V_Alpha, V_Beta, &V_A, &V_B, &V_C);
    PWM->Update_DTC(V_A, V_B, V_C);
    }
    
    
void CurrentRegulator::Commutate(){
    count += 1;
    //GPIOC->ODR = (1 << 4); //Toggle pin for debugging
    theta_elec = _PositionSensor->GetElecPosition();
    _PositionSensor->GetMechPosition();
    SampleCurrent(); //Grab most recent current sample
    Update();   //Run control loop
    SetVoltage();   //Set inverter duty cycles
    //GPIOC->ODR = (0 << 4); //Toggle pin for debugging

    
    if (count==1000){
        //printf("%f\n\r", V_A);
        count = 0;
        }

    
    
      
      }
