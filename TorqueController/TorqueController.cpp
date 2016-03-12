
//cogging torque and torque ripple compensation will go here later

#include "CurrentRegulator.h"
#include "TorqueController.h"

TorqueController::TorqueController(float Kt, CurrentRegulator *regulator)
    {
    _CurrentRegulator = regulator;
    _Kt = Kt;
    
    }

void TorqueController::SetTorque(float torque)
    {
        SetCurrent(0, torque/_Kt);
    }

void TorqueController::SetCurrent(float Id, float Iq)
    {
        _CurrentRegulator->UpdateRef(Id, Iq);
        _CurrentRegulator->Commutate();
        
    }