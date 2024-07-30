#include "slaveRoda.h"

slaveRoda::slaveRoda(BMAktuatorKRAI *BM_cmd, BMAktuatorKRAI *BM_fb, BMAktuatorKRAI *master, int motorNumber, Motor *motor, encoderKRAI *encMotor, pidLo *pidMotor, MovingAverage *movAvg, int ppr_, uint32_t encTimeSampling, float jari_jari_roda)
{
    this->BM_cmd = BM_cmd;
    this->BM_fb = BM_fb;
    this->master = master;
    this->motorNumber = motorNumber;
    this->motor = motor;
    this->encMotor = encMotor;
    this->pidMotor = pidMotor;
    this->movAvg = movAvg;
    this->ppr_ = ppr_;
    this->timeSampling = encTimeSampling;
    this->prevPulses = this->encMotor->getPulses();
    this->jari_jari_roda = jari_jari_roda;
    this->motorPWM = 0.0f;
    this->rpsToSpeed = 2.0f * PI * this->jari_jari_roda;
}

float slaveRoda::getTargetRps()
{
    return this->targetRps_;
}

float slaveRoda::getRps()
{
    return this->RPS;
}

float slaveRoda::getSpeed()
{
    return this->speed;
}

int slaveRoda::getPulses()
{
    return this->encMotor->getPulses();
}

float slaveRoda::getPWM()
{
    return this->motorPWM;
}

void slaveRoda::setTunings(float kp, float ki, float kd)
{
    this->pidMotor->setTunings(kp, ki, kd);
}

void slaveRoda::controlMotorSpeed(float targetSpeed, bool doSampling)
{
    this->targetRps_ = targetSpeed / this->rpsToSpeed;
    if(doSampling){
        this->rodaSamp();
    }
}

void slaveRoda::controlMotorRps(float targetRps, bool doSampling)
{
    this->targetRps_ = targetRps;
    if(doSampling){
        this->rodaSamp();
    }
}

void slaveRoda::encMotorSamp()
{
    this->RPS = this->movAvg->movingAverage((float)(this->encMotor->getPulses() - this->prevPulses)) / ((float)(this->ppr_ * this->timeSampling) / 1000000.0f ); //use this if you trust timeSampling
    // this->omega = 2.0f * PI * this->movAvg->movingAverage((float)(this->encMotor->getPulses() - this->prevPulses)) / ((float)(this->ppr_ * (us_ticker_read() - this->lastSampling)) / 1000000.0f ); //use this if you don't trust timeSampling
    // this->lastSampling = us_ticker_read();
    this->speed = this->RPS * this->rpsToSpeed;
    this->prevPulses = this->encMotor->getPulses();
}

void slaveRoda::pidMotorSamp()
{
    this->motorPWM = this->pidMotor->createpwm(this->targetRps_, this->RPS, max_pwm);
}

void slaveRoda::motorSamp()
{
    if(!this->brakingState_) this->motor->speed(this->motorPWM);
}

void slaveRoda::rodaSamp()
{
    this->encMotorSamp();
    this->pidMotorSamp();
    this->motorSamp();
}

void slaveRoda::forcebrake(bool brakingState)
{
    if(this->brakingState_ != brakingState){
        this->brakingState_ = brakingState;
        if(this->brakingState_){
            this->motor->forcebrake();
        }
    }
}

void slaveRoda::reset(){
    this->pidMotor->reset();
}

void slaveRoda::CANupdate()
{
    if(this->motorNumber == 1){
        this->forcebrake(this->BM_cmd->getSwitch1());
        this->controlMotorRps((float)this->BM_cmd->getMotor1() / 1000.0f, false);
        if(this->resetBoolLast != this->BM_cmd->getSwitch4()){
            this->resetBoolLast = this->BM_cmd->getSwitch4();
            this->reset();
        }
    } else if(this->motorNumber == 2){
        this->forcebrake(this->BM_cmd->getSwitch2());
        this->controlMotorRps((float)this->BM_cmd->getMotor2() / 1000.0f, false);
        if(this->resetBoolLast != this->BM_cmd->getSwitch5()){
            this->resetBoolLast = this->BM_cmd->getSwitch5();
            this->reset();
        }
    } else{
        this->forcebrake(this->BM_cmd->getSwitch3());
        this->controlMotorRps((float)this->BM_cmd->getInteger() / 1000.0f, false);
        if(this->resetBoolLast != this->BM_cmd->getSwitch6()){
            this->resetBoolLast = this->BM_cmd->getSwitch6();
            this->reset();
        }
    }
}

void slaveRoda::sendDebug()
{
    if(this->motorNumber == 1){
        this->BM_fb->setMotor1(this->RoundToInt16(this->RPS * 1000.0f));
    } else if(this->motorNumber == 2){
        this->BM_fb->setMotor2(this->RoundToInt16(this->RPS * 1000.0f));
    } else{
        this->BM_fb->setInteger(this->RoundToInt16(this->RPS * 1000.0f));
    }
}

int16_t slaveRoda::RoundToInt16(float n)
{
    if(n>=0.0f){
        return (int16_t)(n + 0.5f);
    } else{
        return (int16_t)(n - 0.5f);
    }
}