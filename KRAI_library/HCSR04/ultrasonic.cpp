#include "ultrasonic.h"
#include <chrono>
#include <ratio>

Ultrasonic::Ultrasonic(PinName trigger,PinName echo,int time): _echo(echo),_trigger(trigger){

    _sampling_time = time;
}

void Ultrasonic::Echolocation (void){
    
    pulse_width.reset();
    
    _trigger = 1;
    wait_us(10);
    _trigger = 0;

    while (_echo == 0);

    pulse_width.start();
    while (_echo == 1);
    pulse_width.stop();

    _tiempo_transcurrido_us = pulse_width.elapsed_time().count();
    _tiempo_transcurrido_us /= 58;

    ThisThread::sleep_for(std::chrono::milliseconds(_sampling_time));
}

int Ultrasonic::get_measurement(void){
    Ultrasonic::Echolocation ();
    return _tiempo_transcurrido_us;
}

void Ultrasonic::set_SamplingTime(int time){
    _sampling_time = time;
}

void Ultrasonic::print_measurement (void){

    Ultrasonic::Echolocation ();

    printf("Distance : %d\n",_tiempo_transcurrido_us);

}