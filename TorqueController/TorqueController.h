#ifndef TORQUECONTROLLER_H
#define TORQUECONTROLLER_H

#include "CurrentRegulator.h"

class TorqueController{
public:
    TorqueController(float Kt, CurrentRegulator *regulator);
    virtual void SetTorque(float torque);

private:    
    virtual void SetCurrent(float Id, float Iq);
    CurrentRegulator* _CurrentRegulator;
    float _Kt;
    };
    
    

#endif