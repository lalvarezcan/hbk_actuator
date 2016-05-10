#include "TorqueController.h"
#include "ImpedanceController.h"
#include "CurrentRegulator.h"
#include "PositionSensor.h"


ImpedanceController::ImpedanceController(TorqueController *torqueController, PositionSensor *sensor_pos, PositionSensor *sensor_vel){
    _torqueController = torqueController;
    _sensor_pos = sensor_pos;
    _sensor_vel = sensor_vel;
    }

 void ImpedanceController::SetImpedance(float K, float B, float ref){
    float position = _sensor_pos->GetMechPosition();
    float velocity = _sensor_vel->GetMechVelocity();
    float error = ref-position;
    float output = K*error + B*velocity;
    
    _torqueController->SetTorque(output);
    
    

    }
    
