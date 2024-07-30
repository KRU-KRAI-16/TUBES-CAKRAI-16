#include "pidLo.h"

pidLo::pidLo(float p, float i, float d, float _Ts, float _max_out, float _vff, float _freq_lpf, float _max_integral)
: ts(_Ts), max_out(_max_out), vff(_vff), freq_lpf(_freq_lpf), max_integral(_max_integral), kp(p), ki(i), kd(d)
{
    last_error = 0.0;
    integ = 0;
    deriv = 0;
}

void pidLo::setTunings(float p, float i, float d)
{
    kp = p;
    ki = i;
    kd = d;
}

void pidLo::setKp(float p){
    kp = p;
}

void pidLo::setKd(float d){
    kd = d;
}

float pidLo::createpwm_Bound(float setpoint, float feedback, float max, float boundary)
{
    error = setpoint - feedback;

    // get integral (bilinear transform)
    float prop = kp*error;
    if (fabs(setpoint - feedback) < boundary){
        max_integral = fabs(max) - fabs(prop);
        integ += ki*(error + last_error) * ts / 2;
        if (fabs(integ) > fabs(max_integral))
        {
            integ = fabs(max_integral) * (fabs(integ) / integ);
        }
    }
    else {
        max_integral = 0.0;
    }
    
    
    // get derivatif with freq_lpf (bilinear transform)
    // y(n) = (2*(x(n)-x(n-1) - (y(n-1)(t-2N)))) / (t+2N)
    deriv = kd*(error - last_error) / ts;

    last_error = error;

    last_out = prop + integ + deriv + vff * setpoint;
    if (fabs(last_out) > fabs(max))
    {
        last_out = fabs(max) * (fabs(last_out) / last_out);
    }

    return last_out;
}

float pidLo ::createpwm(float setpoint, float feedback, float max)
{
    error = setpoint - feedback;

    // get integral (bilinear transform)
    float prop = kp*error;
    if (fabs(max)>fabs(prop)){
        max_integral = fabs(max) - fabs(prop);
        integ += ki*(error + last_error) * ts / 2;
        if (fabs(integ) > fabs(max_integral))
        {
            integ = fabs(max_integral) * (fabs(integ) / integ);
        }
    }
    else {
        max_integral = 0.0;
    }
    
    
    // get derivatif with freq_lpf (bilinear transform)
    // y(n) = (2*(x(n)-x(n-1) - (y(n-1)(t-2N)))) / (t+2N)
    deriv = kd*(error - last_error) / ts;

    last_error = error;

    last_out = prop + integ + deriv + vff * setpoint;
    if (fabs(last_out) > fabs(max))
    {
        last_out = fabs(max) * (fabs(last_out) / last_out);
    }
    return last_out;
}

void pidLo :: reset(){
    integ = 0;
    deriv = 0;
    last_error = 0;
    last_out = 0;
}