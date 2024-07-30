#include "masterRoda.h"

masterRoda::masterRoda(BMAktuatorKRAI *BM_cmd, BMAktuatorKRAI *BM_fb, BMAktuatorKRAI *master, int motorNumber, float jari_jari_roda)
{
    this->BM_cmd = BM_cmd;
    this->BM_fb = BM_fb;
    this->master = master;
    this->motorNumber = motorNumber;
    this->jari_jari_roda = jari_jari_roda;
    this->speedToRps = 1.0f/(2.0f * PI * this->jari_jari_roda);
    this->forcebrake(false);
}

float masterRoda::getTargetRps()
{
    
    return this->targetRps;
}


float masterRoda::getRps()
{
    if(this->motorNumber == 1){
        return (float)this->BM_fb->getMotor1() / 1000.0f;
    } else if(this->motorNumber == 2){
        return (float)this->BM_fb->getMotor2() / 1000.0f;
    } else{
        return (float)this->BM_fb->getInteger() / 1000.0f;
    }
    
}

// float masterRoda::getSpeed()
// {
//     return this->speed;
// }

// int masterRoda::getPulses()
// {
//     return this->encMotor->getPulses();
// }

// float masterRoda::getPWM()
// {
//     return this->motorPWM;
// }

// void masterRoda::setTunings(float kp, float ki, float kd)
// {
//     this->pidMotor->setTunings(kp, ki, kd);
// }

void masterRoda::controlMotorSpeed(float targetSpeed, bool doSampling)
{
    this->targetRps = targetSpeed * this->speedToRps;
    int16_t RPSSent = this->RoundToInt16(this->targetRps * 1000.0f);
    if(this->motorNumber == 1){
        this->BM_cmd->setMotor1(RPSSent);
    } else if(this->motorNumber == 2){
        this->BM_cmd->setMotor2(RPSSent);
    } else{
        this->BM_cmd->setInteger(RPSSent);
    }
    if(doSampling){
        if (this->BM_cmd->sendingCAN(this->master->getNoBM()))  { };
    }
}

void masterRoda::forcebrake(bool brakingState)
{
    if(this->brakingState_ != brakingState){
        this->brakingState_ = brakingState;
        if(this->motorNumber == 1){
            this->BM_cmd->setSwitch1(this->brakingState_);
        } else if(this->motorNumber == 2){
            this->BM_cmd->setSwitch2(this->brakingState_);
        } else{
            this->BM_cmd->setSwitch3(this->brakingState_);
        }
    }
}

void masterRoda::reset(){
    this->resetBoolLast = !this->resetBoolLast;
    if(this->motorNumber == 1){
        this->BM_cmd->setSwitch4(this->resetBoolLast);
    } else if(this->motorNumber == 2){
        this->BM_cmd->setSwitch5(this->resetBoolLast);
    } else{
        this->BM_cmd->setSwitch6(this->resetBoolLast);
    }
}

int16_t masterRoda::RoundToInt16(float n)
{
    if(n>=0.0f){
        return (int16_t)(n + 0.5f);
    } else{
        return (int16_t)(n - 0.5f);
    }
}