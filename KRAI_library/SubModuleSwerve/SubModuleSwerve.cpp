#include "SubModuleSwerve.h"

SubModuleSwerve::SubModuleSwerve(Motor *motorSudut, encoderKRAI *encMotorSudut, pidLo *pidMotorSudut)
{
    this->motorSudut = motorSudut;
    this->encMotorSudut = encMotorSudut;
    this->pidMotorSudut = pidMotorSudut;
    // this->pidMotorSudut->setInputLimits(-90.0f / 360.f * PPR, 90.0f / 360.0f * PPR);
    // this->pidMotorSudut->setOutputLimits(-1.0, 1.0);
}

void SubModuleSwerve::setTunings(float kp, float ki, float kd)
{
    // float tau_i = (ki < 0) ? 0.0f : kp / ki;
    // float tau_d = (kp < 0) ? 0.0f : kd / kp;
    this->pidMotorSudut->setTunings(kp, ki, kd);
}

float SubModuleSwerve::normalizeAngle(float theta)
{
    float normalisasi = std::fmod(theta, 360); // fmod adalah fungsi untuk melakukan modulo | harus include cmath
    return (normalisasi < 0) ? (360 + normalisasi) : normalisasi;
}

float SubModuleSwerve::optimizeMotorDirection(float targetAngle)
{
    float arahRoda_A_gauge = 360 * this->encMotorSudut->getPulses() / (float)PPR;
    float arahRoda_A = std::fabs(normalizeAngle(arahRoda_A_gauge));
    float arahRoda_B = normalizeAngle(arahRoda_A + 180);

    float deltaTheta1_A = targetAngle - arahRoda_A;
    float deltaTheta2_A = 360 - std::fabs(deltaTheta1_A);
    float setPointDisplacement_A = (std::fabs(deltaTheta1_A) < deltaTheta2_A) ? std::fabs(deltaTheta1_A) : deltaTheta2_A;

    float deltaTheta1_B = targetAngle - arahRoda_B;
    float deltaTheta2_B = 360 - std::fabs(deltaTheta1_B);
    float setPointDisplacement_B = (std::fabs(deltaTheta1_B) < deltaTheta2_B) ? std::fabs(deltaTheta1_B) : deltaTheta2_B;
    float setPointDisplacement;
    float arahRoda;

    if (setPointDisplacement_A <= setPointDisplacement_B)
    {
        this->fwdBwd = 1;
        setPointDisplacement = setPointDisplacement_A;
        arahRoda = arahRoda_A;
    }
    else
    {
        this->fwdBwd = -1;
        setPointDisplacement = setPointDisplacement_B;
        arahRoda = arahRoda_B;
    }

    if (normalizeAngle(setPointDisplacement + arahRoda) == targetAngle) // if true => rotate CW
    {
        return setPointDisplacement;
    }
    else
    {
        return -1 * setPointDisplacement;
    }
}

void SubModuleSwerve::controlDirMotor(float targetAngle)
{
    float motorDirectionInPulses = this->encMotorSudut->getPulses() + optimizeMotorDirection(targetAngle) / 360.0f * PPR;
    this->motorAnglePWM = this->pidMotorSudut->createpwm(motorDirectionInPulses, this->encMotorSudut->getPulses(), 1.0f);
    this->motorSudut->speed(this->motorAnglePWM);
    // this->motorSudut->speed(0.4);
}