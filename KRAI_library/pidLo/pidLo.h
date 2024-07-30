#ifndef _PIDLO_H_
#define _PIDLO_H_

#include "math.h"
#include "mbed.h"

class pidLo {
    private:
        float kp;
        float ki;
        float kd;
        float ts;
        float vff;
        float freq_lpf;
        float max_integral;
        float max_out;

        float error;
        float integ;
        float deriv;

        float last_error;
        float last_out;
    public:
        pidLo(float p, float i, float d, float _Ts, float _max_out, float _vff, float _freq_lpf, float _maxIntegral);
        float createpwm(float setpoint, float feedback, float max);
        float createpwm_Bound(float setpoint, float feedback, float max, float boundary);
        void setTunings(float p, float i, float d);
        void setKp(float p);
        void setKd(float d);
        void reset();
        float getPParam(){ return this->kp; }
        float getIParam(){ return this->ki; }
        float getDParam(){ return this->kd; }
};

#endif