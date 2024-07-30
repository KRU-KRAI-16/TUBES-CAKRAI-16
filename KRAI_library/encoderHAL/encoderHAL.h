#ifndef ENCODER_H
#define ENCODER_H

#include "mbed.h"

namespace mbed{
    class encoderHAL {
        private :
            TIM_Encoder_InitTypeDef encoder;
            TIM_HandleTypeDef timer;
            TIM_TypeDef * TIM;
        public :
            encoderHAL(TIM_TypeDef * _TIM);
            encoderHAL(TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode);
            encoderHAL(TIM_Encoder_InitTypeDef * _encoder, TIM_HandleTypeDef * _timer, TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode);
            int32_t getPulses(bool reset);
            TIM_HandleTypeDef* GetTimer();
            int getDistance(bool reset);
    };
}// namespace mbed

#endif