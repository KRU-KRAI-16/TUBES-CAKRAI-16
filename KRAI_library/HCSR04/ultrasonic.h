#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "mbed.h"
#include <string>

class Ultrasonic{
public:
    Ultrasonic(PinName trigger,PinName echo,int time);
    int get_measurement(void); 
    void set_SamplingTime(int time);
    void print_measurement (void);

private:
    void Echolocation (void);
    DigitalOut _trigger;
    DigitalIn _echo;
    Timer pulse_width;
    int _tiempo_transcurrido_us;
    int _sampling_time;
};

#endif