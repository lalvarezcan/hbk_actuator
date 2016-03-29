#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H

class PositionSensor {
public:
    virtual float GetMechPosition() {return 0.0f;}
    virtual float GetElecPosition() {return 0.0f;}
};
  
  
class PositionSensorEncoder: public PositionSensor {
public:
    PositionSensorEncoder(int CPR, float offset);
    virtual float GetMechPosition();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual float GetElecVelocity();
private:
    InterruptIn *ZPulse;
    DigitalIn *ZSense;
    //DigitalOut *ZTest;
    virtual void ZeroEncoderCount(void);
    virtual void ZeroEncoderCountDown(void);
    int _CPR, flag, rotations;
    //int state;
    float _offset, MechPosition, dir, test_pos;
};

#endif