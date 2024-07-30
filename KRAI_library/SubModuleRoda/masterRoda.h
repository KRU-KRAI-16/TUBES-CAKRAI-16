//Library ini menggabung semua library yang dibutuhkan untuk kontrol sebuah roda (speed) menggunakan brushed DC motor, encoderKRAI, pidLo, dan MovingAverage

#pragma once

#include "../CanBusKRAI/BMAktuatorKRAI.hpp"
// #include <cmath>

#define max_pwm 0.8f

#ifndef PI
#define PI 3.141592653589f
#endif

class masterRoda
{
private:
    BMAktuatorKRAI *BM_cmd;
    BMAktuatorKRAI *BM_fb;
    BMAktuatorKRAI *master;

    float jari_jari_roda;
    int motorNumber;
    float speedToRps;
    bool brakingState_;
    float targetRps;
    float RPS;
    bool resetBoolLast = false;
    int16_t RoundToInt16(float n);

public:
    masterRoda(BMAktuatorKRAI *BM_cmd, BMAktuatorKRAI *BM_fb, BMAktuatorKRAI *master, int motorNumber, float jari_jari_roda);
    
    //control
    void controlMotorSpeed(float targetSpeed, bool doSampling); //in m/s

    //procedure
    void forcebrake(bool brakingState); //sebaiknya selalu dipanggil setiap repetisi
    void reset(); //toggle reset switch

    //setter
    // void setTunings(float kp, float ki, float kd);

    //getter
    float getTargetRps();
    // float getSpeed();
    float getRps();
    // int getPulses();
    // float getPWM();
    // float getPParam(){ return this->pidMotor->getKp(); };
    // float getIParam(){ return this->pidMotor->getKi(); };
    // float getDParam(){ return this->pidMotor->getKd(); };
};