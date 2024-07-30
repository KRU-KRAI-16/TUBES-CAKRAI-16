#pragma once
#include "mbed.h"
#include "../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_library/Motor/Motor.h"
#include "../../KRAI_library/PIDAaronBerk/PIDAaronBerk.h"
#include "../../KRAI_library/pidLo/pidLo.h"
#include <cmath>
#define PPR 538

class SubModuleSwerve
{
private:
    Motor *motorSudut;
    encoderKRAI *encMotorSudut;
    pidLo *pidMotorSudut;

    int fwdBwd; // Forward: 1 || Backward: -1
    float motorAnglePWM;

public:
    SubModuleSwerve(Motor *motorSudut, encoderKRAI *encMotorSudut, pidLo *pidMotorSudut);
    float normalizeAngle(float theta);
    float optimizeMotorDirection(float targetAngle); // target arah dari roda swerve drive (sudah ternormalisasi [0, 360) derajat)
    void controlDirMotor(float targetAngle);
    void setTunings(float kp, float ki, float kd);
    float getPParam(){ return this->pidMotorSudut->getPParam(); };
    float getIParam(){ return this->pidMotorSudut->getIParam(); };
    float getDParam(){ return this->pidMotorSudut->getDParam(); };
    float getPWMMotorAngle(){ return this->motorAnglePWM; };
};