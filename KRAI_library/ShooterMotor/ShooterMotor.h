#pragma once

#include "mbed.h"
#include "../KRAI_Library/encoderKRAI/encoderKRAI.h"
#include "../KRAI_Library/Motor/Motor.h"
#include "../KRAI_Library/pidLo/pidLo.h"
#include "../KRAI_Library/MovingAverage/MovingAverage.h"
#include "../Configurations/Constants.h"

// #define PPR_LM 3358

class ShooterMotor
{
private:
    // Left Motor
    Motor *flywheelMotor;
    encoderKRAI *encflywheelMotor;
    pidLo *pidflywheelMotor;

    // Moving Average
    MovingAverage *movAvgLM;
    MovingAverage *movAvgAccel;

    // Target
    uint32_t prevTimeNow;
    float setPointFlywheel;
    float maxRPMLM; // Revolution per Minute
    float maxRPMRM; // Revolution per Minute

    float outputPMW_LM;
    float omegaLM;  // Revolutions per Minute
    float prevOmega;
    int prevPulsesLM;

    int prevPulsesReloader;
    bool isInReload;
    float accelShooter;

public:
    ShooterMotor(Motor *flywheelMotor,encoderKRAI *encflywheelMotor,pidLo *pidflywheelMotor,MovingAverage *movAvgLM, MovingAverage *movAvgAccel, Motor *motorReload, encoderKRAI *encMotorReload);
    // ShooterMotor(Motor *flywheelMotor, Motor *rightMotor, encoderKRAI *encflywheelMotor, encoderKRAI *encRightMotor,
    //             pidLo *pidflywheelMotor, pidLo *pidRightMotor, MovingAverage *movAvgLM, MovingAverage *movAvgRM);

    void controlOmegaShooter(float setPoint);
    void setRPM(float rpm){ this->flywheelMotor->speed(rpm);}
    void setTuningLM(float kp, float ki, float kd);

    float getOmegaShooter(){ return this->omegaLM; }
    float getAccelShooter(){ return this->accelShooter; }
    float getSetpoint(){ return this->setPointFlywheel; }

    float getPParamLM(){ return this->pidflywheelMotor->getPParam(); }
    float getIParamLM(){ return this->pidflywheelMotor->getIParam(); }
    float getDParamLM(){ return this->pidflywheelMotor->getDParam(); }

    void setParamSetpoint(float updateSetpoint) { this->setPointFlywheel = updateSetpoint;}
    

    void runReloader(float deltaDerajatRLD, float pwmReloader);
    void setReloaderStatus(bool flag) {this->isInReload = flag; }
    bool getReloaderStatus(){ return this->isInReload; }

    float getCalcRPM(float xr, int angle);
};
