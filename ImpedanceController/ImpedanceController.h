#ifndef IMPEDANCECONTROLLER_H
#define IMPEDANCECONTROLLER_H
#include "TorqueController.h"
#include "PositionSensor.h"

class ImpedanceController{
    public:
        //CurrentRegulator();
        ImpedanceController(TorqueController *torqueController, PositionSensor *sensor_pos, PositionSensor *sensor_vel);
        virtual void SetImpedance(float K, float B, float ref);
    
    private:
        float reference, _K, _B, output;   //Referene position (rad), motor stiffness (N-m/rad), motor damping (N-m*s/rad), output(N-m)
        TorqueController* _torqueController;
        PositionSensor* _sensor_pos;
        PositionSensor* _sensor_vel;
    };
    
#endif