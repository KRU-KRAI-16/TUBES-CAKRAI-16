#include "ShooterMotor.h"

ShooterMotor::ShooterMotor(Motor *flywheeFlyotor,encoderKRAI *encflywheeFlyotor,pidLo *pidflywheeFlyotor,MovingAverage *movAvgFly, MovingAverage *movAvgAccel,  uint32_t *millis)
{
    this->millis = millis;

    this->flywheeFlyotor = flywheeFlyotor;
    this->encflywheeFlyotor = encflywheeFlyotor;
    this->pidflywheeFlyotor = pidflywheeFlyotor;
    this->movAvgFly = movAvgFly;
    this->movAvgAccel = movAvgAccel;

    this->prevTimeNow = 0;
    this->accelShooter = 0;

    this->outputPMW_Fly = 0;
    this->prevPulsesFly = 0;
    this->omegaFly = 0;
    this->prevOmega = 0;
}


void ShooterMotor::setTuningFly(float kp, float ki, float kd)
{
    this->pidflywheeFlyotor->setTunings(kp, ki, kd);
    
    printf("Set to %f %f %f \n", kp, ki, kd);
    printf("Pakai Fungsi %f %f %f\n", getPParamFly(), getIParamFly(), getDParamFly());
}

void ShooterMotor::controlOmegaShooter(float setPoint)
{
    this->setPointFlywheel = setPoint;

    uint32_t timeNow = *this->millis;
    /*
    Speed Measurement
        FREQUENCY-BASED MEASUREMENT
        measured the number of encoder pulses in a fix gate time
        omega = (delta_pulses) / (PPR * timeSampling)
    */
    float beforeMovAvg;
    this->omegaFly = ((this->encflywheeFlyotor->getPulses() - this->prevPulsesFly) / (PPR_FLYWHEEL * (float)(timeNow - this->prevTimeNow)/1000.0f)) * 60; // Revolutions per Minute
    beforeMovAvg = this->omegaFly;
    this->omegaFly = this->movAvgFly->movingAverage(this->omegaFly);

    // PID dan set speed motor
    this->outputPMW_Fly = this->pidflywheeFlyotor->createpwm(setPoint, this->omegaFly, 1.0); // PWM POSITIVE => omegaFly (+)

    // PID Hanya jika diatas 0
    if (setPoint > 0)
    {
        this->flywheeFlyotor->speed(this->outputPMW_Fly);
    } else {
        this->flywheeFlyotor->speed(0.0);
        this->pidflywheeFlyotor->reset();
    }
    
    // this->flywheeFlyotor->speed(0.8f);

    // Akselerasi omega
    this->accelShooter = (this->omegaFly - this->prevOmega)/(60* (float)(timeNow - this->prevTimeNow)/1000.0f);
    this->accelShooter = this->movAvgAccel->movingAverage(this->accelShooter);

    // printf("%f %f %f\n", this->omegaFly, this->accelShooter, (this->omegaFly - this->prevOmega)/(60* (float)(timeNow - this->prevTimeNow)/1000000.0f));
    // printf("%f %f\n", this->omegaFly, this->accelShooter);

    // Update nilai pulses
    this->prevPulsesFly = this->encflywheeFlyotor->getPulses();
    this->prevTimeNow = *this->millis;
    this->prevOmega = this->omegaFly;


    // printf("%f %f\n", beforeMovAvg, this->omegaFly);

    /* BUAT TUNNING LAPANGAN */
    //printf(" %f %f ", this->omegaFly, setPoint);

    // printf("%f %f %f %f %f\n", this->omegaFly, setPoint, getPParamFly(), getIParamFly(), getDParamFly());

    // printf("%d %d\n", this->encflywheeFlyotor->getPulses(), this->encRightMotor->getPulses());
    // printf("%f %f\n", this->omegaFly, this->omegaRM);
    // printf("%f %f %f %f %f %f\n", this->omegaFly/maxRPMFly, setPoint/maxRPMFly, this->outputPMW_Fly, getPParamFly(), getIParamFly(), getDParamFly());
    // printf("%f %f %f %f %f\n", this->omegaRM, setPoint, getPParamRM(), getIParamRM(), getDParamRM());
    // printf("%f %f %f %f %f %f\n", this->omegaRM/maxRPMRM, setPoint/maxRPMRM, this->outputPWM_RM, getPParamRM(), getIParamRM(), getDParamRM());
    // printf("%f %f %f\n", this->omegaRM/maxRPM, setPoint/maxRPM, this->outputPWM_RM);
}


float ShooterMotor::getCalcRPM(float x, int targetAngle)
{
    if (targetAngle == 55)
    {
        return (-104.4*x*x + 827.3*x + 1391);
    } else if (targetAngle == 65)
    {
        return (245.9*x*x + 655.5*x + 1259);
    } else if (targetAngle == 70)
    {
        return (-745.1*x*x + 1021*x + 2070);
    } else {
        return 0.0;
    }
}
