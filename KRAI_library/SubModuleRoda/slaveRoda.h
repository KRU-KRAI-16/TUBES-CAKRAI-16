//Library ini menggabung semua library yang dibutuhkan untuk kontrol sebuah roda (speed) menggunakan brushed DC motor, encoderKRAI, pidLo, dan MovingAverage

#pragma once
#include "../CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../encoderKRAI/encoderKRAI.h"
#include "../Motor/Motor.h"
#include "../pidLo/pidLo.h"
#include "../MovingAverage/MovingAverage.h"
// #include <cmath>

#define max_pwm 0.8f

#ifndef PI
#define PI 3.141592653589
#endif

class slaveRoda
{
private:
    BMAktuatorKRAI *BM_cmd;
    BMAktuatorKRAI *BM_fb;
    BMAktuatorKRAI *master;
    
    Motor *motor;
    encoderKRAI *encMotor;
    pidLo *pidMotor;
    MovingAverage *movAvg;

    int ppr_;
    float jari_jari_roda;
    int motorNumber;
    int prevPulses;
    uint32_t timeSampling;
    uint32_t lastSampling;
    float motorPWM;
    float RPS;
    float speed;
    float targetRps_;
    float rpsToSpeed;
    bool brakingState_ = false;
    bool resetBoolLast = false;
    int16_t RoundToInt16(float n);

public:
    //@param encTimeSampling timesampling in us
    slaveRoda(BMAktuatorKRAI *BM_cmd, BMAktuatorKRAI *BM_fb, BMAktuatorKRAI *master, int motorNumber, Motor *motor, encoderKRAI *encMotor, pidLo *pidMotor, MovingAverage *movAvg, int ppr_, uint32_t encTimeSampling, float jari_jari_roda);
    
    //control
    void controlMotorSpeed(float targetSpeed, bool doSampling);
    void controlMotorRps(float targetRps, bool doSampling);
    
    //update from CAN
    void CANupdate();
    void sendDebug(); //make memory integer 

    //sampling
    void rodaSamp();
    void encMotorSamp();
    void pidMotorSamp();
    void motorSamp();

    //procedure
    void forcebrake(bool brakingState);
    void reset();

    //setter
    void setTunings(float kp, float ki, float kd);

    //getter
    float getTargetRps();
    float getSpeed();
    float getRps();
    int getPulses();
    float getPWM();
    float getPParam(){ return this->pidMotor->getPParam(); };
    float getIParam(){ return this->pidMotor->getIParam(); };
    float getDParam(){ return this->pidMotor->getDParam(); };
};