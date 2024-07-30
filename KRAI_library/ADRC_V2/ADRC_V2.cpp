#include "ADRC_V2.h"

ADRC_V2::ADRC_V2(float tSampling, float b0, float tSettle, float gain_zESO, float integralInc)
{
    this->tSampling = tSampling;
    this->x1 = 0;
    this->x2 = 0;
    this->x3 = 0;
    this->uInput = 0;
    this->b0 = b0;
    this->h0 = tSampling;
    this->gain_z_eso = gain_zESO;
    this->integralInc = integralInc;

    this->Tsettle = tSettle; // second
    this->s_cl = -6.0 / Tsettle;
    this->z_eso = exp(this->gain_z_eso * s_cl * tSampling);
    this->l1 = 1 - pow(z_eso, 3);
    this->l2 = 3.0f / (2 * tSampling) * pow((1 - z_eso), 2) * (1 + z_eso);
    this->l3 = 1.0f / pow(tSampling, 2) * pow((1 - z_eso), 3);
    this->Kp = s_cl * s_cl;
    this->Kd = -2 * s_cl;

    this->Ad << 1, tSampling, pow(tSampling, 2)/2.0f,
                0, 1, tSampling,
                0, 0, 1;
    this->Bd << b0 * pow(tSampling, 2) / 2.0f, b0 * tSampling, 0;
    this->Cd << 1, 0, 0;
    this->Lc << l1, l2, l3;

    this->A_eso = Ad - Lc * Cd * Ad;
    // this->B_eso = this->supplyVoltage * (Bd - Lc * Cd * Bd); // Perhatikan voltase yang dimasukkan ke dalam motor
    this->B_eso = (Bd - Lc * Cd * Bd); // Perhatikan voltase yang dimasukkan ke dalam motor
    this->trajectorySp = 0;
    this->derivativeTrajectory = 0;
    this->offsetPWM_on = true;
}

void ADRC_V2::changeTuningParam(float b0, float tSettle, float gain_z_eso, float integralInc)
{
    this->b0 = b0;
    this->Tsettle = tSettle;
    this->gain_z_eso = gain_z_eso;
    this->integralInc = integralInc;
    
    this->s_cl = -6.0 / Tsettle;
    this->z_eso = exp(this->gain_z_eso * s_cl * tSampling);
    this->l1 = 1 - pow(z_eso, 3);
    this->l2 = 3.0f / (2 * tSampling) * pow((1 - z_eso), 2) * (1 + z_eso);
    this->l3 = 1.0f / pow(tSampling, 2) * pow((1 - z_eso), 3);
    this->Kp = s_cl * s_cl;
    this->Kd = -2 * s_cl;

    this->Ad << 1, tSampling, pow(tSampling, 2)/2.0f,
                0, 1, tSampling,
                0, 0, 1;
    this->Bd << b0 * pow(tSampling, 2) / 2.0f, b0 * tSampling, 0;
    this->Cd << 1, 0, 0;
    this->Lc << l1, l2, l3;

    this->A_eso = Ad - Lc * Cd * Ad;
    // this->B_eso = this->supplyVoltage * (Bd - Lc * Cd * Bd); // Perhatikan voltase yang dimasukkan ke dalam motor
    this->B_eso = (Bd - Lc * Cd * Bd); // Perhatikan voltase yang dimasukkan ke dalam motor
}

float ADRC_V2::sign(float x)
{
    if (x > 0) {
        return 1;
    }
    else if (x < 0) {
        return -1;
    }
    else {
        return 0;
    }
}

float ADRC_V2::fhan(float v1, float v2, float r0, float h0)
{
    float d = r0 * h0 * h0;
    float a0 =  h0 * v2;
    float y = v1 + a0;
    float a1 = sqrt(d * (d + 8 * fabs(y)));
    float a2 = a0 + sign(y) * (a1 - d) / 2.0f;
    float sy = (sign(y + d) - sign(y - d)) / 2.0f;
    float a = (a0 + y - a2) * sy + a2;
    float sa = (sign(a + d) - sign(a - d)) / 2.0f;
    return -r0 * (a / d - sign(a)) * sa - r0 * sign(a);
}

float ADRC_V2::fhan_setPointTrajectory(float setPoint, float explicit_r0)
{
    if (explicit_r0 == 0)
    {
        this->trajectorySp = setPoint;
        return setPoint;
    }

    this->trajectorySp += this->tSampling * this->derivativeTrajectory;
    this->derivativeTrajectory += this->tSampling * fhan(this->trajectorySp - setPoint, this->derivativeTrajectory, explicit_r0, this->h0);
    return this->trajectorySp;
}

float ADRC_V2::fhan_setPointTrajectory(float setPoint, float explicit_r0, bool autoOff, float speedNow)
{
    if (explicit_r0 == 0)
    {
        this->trajectorySp = setPoint;
        return setPoint;
    }

    if (autoOff) {
        if (fabs(setPoint - speedNow) < 0.1) {
            explicit_r0 = 0;
        }
    }

    this->trajectorySp += this->tSampling * this->derivativeTrajectory;
    this->derivativeTrajectory += this->tSampling * fhan(this->trajectorySp - setPoint, this->derivativeTrajectory, explicit_r0, this->h0);
    return this->trajectorySp;

}


void ADRC_V2::resetADRC()
{
    this->x1 = 0;
    this->x2 = 0;
    this->x3 = 0;
    this->uInput = 0;
}

void ADRC_V2::resetSetpoint()
{
    this->trajectorySp = 0;
    this->derivativeTrajectory = 0;
}


float ADRC_V2::createInputSignal(float setPoint, float yPlant, float maxOutput)
{
    // fhan_setPointTrajectory(setPoint);
    this->trajectorySp = setPoint;
    this->x1 = this->A_eso.coeff(0,0) * this->x1 + this->A_eso.coeff(0, 1) * this->x2 + this->A_eso.coeff(0, 2) * this->x3 + this->l1 * yPlant + this->B_eso.coeff(0, 0) * this->uInput;
    this->x2 = this->A_eso.coeff(1,1) * this->x2 + this->A_eso.coeff(1, 0) * this->x1 + this->A_eso.coeff(1, 2) * this->x3 + this->l2 * yPlant + this->B_eso.coeff(1, 0) * this->uInput;
    this->x3 = this->A_eso.coeff(2,2) * this->x3 + this->A_eso.coeff(2, 1) * this->x2 + this->A_eso.coeff(2, 0) * this->x1 + this->l3 * yPlant + this->B_eso.coeff(2, 0) * this->uInput;
    this->uInput = (((this->Kp + 40) * (this->trajectorySp - this->x1)) - this->Kd * this->x2 - this->x3) / this->b0;

    if (setPoint > -0.001 && setPoint < 0.001 && yPlant > -0.001 && yPlant < 0.001 && this->x2 > -0.001 && this->x2 < 0.001)
    {
        resetADRC();
        this->offsetPWM_on = true;
        return 0;
    }
    else {
        if (this->offsetPWM_on) {
            this->uInput += this->integralInc * sign(setPoint);
        }
        if (yPlant > 0.045 || yPlant < -0.045) {
            this->offsetPWM_on = false;
        }
    }

    if (fabs(this->uInput) > maxOutput)
    {
        this->uInput = maxOutput * sign(this->uInput);
        return maxOutput * sign(this->uInput);
    }
    return this->uInput;
}