#pragma once
#include "mbed.h"
#include "../../KRAI_library/Eigen/Dense"
#include <cmath>

class ADRC_V2
{
private:
    float tSampling;
    float x1;
    float x2;
    float x3;
    float uInput;
    float b0;
    float h0;
    float trajectorySp, derivativeTrajectory;
    Eigen::Matrix3f Ad;
    Eigen::Matrix<float, 3, 1> Bd;
    Eigen::Matrix<float, 1, 3> Cd;
    Eigen::Matrix<float, 3, 1> Lc;
    Eigen::Matrix3f A_eso;
    Eigen::Matrix<float, 3, 1> B_eso;


    // tuning parameter
    float Tsettle; // second
    float s_cl;
    float z_eso;
    float gain_z_eso;
    float l1;
    float l2;
    float l3;
    float Kp;
    float Kd;
    float integralInc;

    bool offsetPWM_on;

public:
    ADRC_V2(float tSampling, float b0, float tSettle, float gain_zESO, float integralInc);
    float fhan(float v1, float v2, float r0, float h0);
    float sign(float x);
    float fhan_setPointTrajectory(float setPoint, float explicit_r0);
    float fhan_setPointTrajectory(float setPoint, float explicit_r0, bool autoOff, float speedNow);
    float getSPTrajectory() { return this->trajectorySp; }
    float getDerivativeSPTraj() { return this->derivativeTrajectory; }
    void setSPTrajectory(float x) { this->trajectorySp = x; }
    void setDerivativeSPTraj(float x) { this->derivativeTrajectory = x; }
    void resetADRC();
    void changeTuningParam(float b0, float tSettle, float gain_z_eso, float integralInc);

    void resetSetpoint();
    float createInputSignal(float setPoint, float yPlant, float maxOutput);
};