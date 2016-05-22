#ifndef POSITIONSENSOR_H
#define POSITIONSENSOR_H
class PositionSensor {
public:
    virtual float GetMechPosition() {return 0.0f;}
    virtual float GetElecPosition() {return 0.0f;}
    virtual float GetMechVelocity() {return 0.0f;}
};
  
  
class PositionSensorEncoder: public PositionSensor {
public:
    PositionSensorEncoder(int CPR, float offset, int ppairs);
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
    int _CPR, flag, rotations, _ppairs;
    //int state;
    float _offset, MechPosition, dir, test_pos, vel_old, out_old;
};

class PositionSensorSPI: public PositionSensor{
public:
    PositionSensorSPI(int CPR, float offset, int ppairs);
    virtual float GetMechPosition();
    virtual float GetElecPosition();
    virtual float GetMechVelocity();
    virtual int GetRawPosition();
    virtual void ZeroPosition();
private:
    float _offset, MechPosition, MechOffset;
    int _CPR, rotations, old_counts, _ppairs;
    SPI *spi;
    DigitalOut *cs;
};
#endif