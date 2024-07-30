#include "encoderHAL.h"

// Constants
#ifndef PI
#define PI  3.14159265359
#endif

#ifndef D_RODA
#define D_RODA 0.06
#endif

namespace mbed
{
    encoderHAL::encoderHAL(TIM_TypeDef * _TIM)
    {
        this->TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        // EncoderInit(&encoder, &timer, _TIM, 0xffff, TIM_ENCODERMODE_TI12);
        this->timer.Instance = _TIM;
        this->timer.Init.Period = 0xffff;
        this->timer.Init.CounterMode = TIM_COUNTERMODE_UP;
        this->timer.Init.Prescaler = 0;
        this->timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

        this->encoder.EncoderMode = TIM_ENCODERMODE_TI12;

        this->encoder.IC1Filter = 0x0F;
        this->encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
        this->encoder.IC1Prescaler = TIM_ICPSC_DIV4;
        this->encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;

        this->encoder.IC2Filter = 0x0F;
        this->encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        this->encoder.IC2Prescaler = TIM_ICPSC_DIV4;
        this->encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

        if (HAL_TIM_Encoder_Init(&this->timer, &this->encoder) != HAL_OK) {
            printf("Couldn't Init Encoder\r\n");
            while (1) {}
        }

        if(HAL_TIM_Encoder_Start(&this->timer, TIM_CHANNEL_1) != HAL_OK) {
            printf("Couldn't Start Encoder\r\n");
            while (1) {}
        }

        // Update (aka over- and underflow) interrupt enabled
        this->TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        this->TIM->SR &= 0xfffe;
        //generate update event
        this->TIM->EGR = 1;
        //enable counter
        this->TIM->CR1 = 1;
    }
    
    encoderHAL::encoderHAL(TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode)
    {
        this->TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        // EncoderInit(&encoder, &timer, _TIM, _maxcount, _encmode);
        this->timer.Instance = _TIM;
        this->timer.Init.Period = _maxcount;
        this->timer.Init.CounterMode = TIM_COUNTERMODE_UP;
        this->timer.Init.Prescaler = 0;
        this->timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

        this->encoder.EncoderMode = _encmode;

        this->encoder.IC1Filter = 0x0F;
        this->encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
        this->encoder.IC1Prescaler = TIM_ICPSC_DIV4;
        this->encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;

        this->encoder.IC2Filter = 0x0F;
        this->encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        this->encoder.IC2Prescaler = TIM_ICPSC_DIV4;
        this->encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

        if (HAL_TIM_Encoder_Init(&this->timer, &this->encoder) != HAL_OK) {
            printf("Couldn't Init Encoder\r\n");
            while (1) {}
        }

        if(HAL_TIM_Encoder_Start(&this->timer, TIM_CHANNEL_1) != HAL_OK) {
            printf("Couldn't Start Encoder\r\n");
            while (1) {}
        }

        // Update (aka over- and underflow) interrupt enabled
        this->TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        this->TIM->SR &= 0xfffe;
    }
    
    encoderHAL::encoderHAL(TIM_Encoder_InitTypeDef * _encoder, TIM_HandleTypeDef * _timer, TIM_TypeDef * _TIM, uint32_t _maxcount, uint32_t _encmode)
    {
        timer = *_timer;
        this->encoder = *_encoder;
        this->TIM = _TIM;
        // Initialisation of the TIM module as an encoder counter
        // EncoderInit(&encoder, &timer, _TIM, _maxcount, _encmode);
        this->timer.Instance = _TIM;
        this->timer.Init.Period = _maxcount;
        this->timer.Init.CounterMode = TIM_COUNTERMODE_UP;
        this->timer.Init.Prescaler = 0;
        this->timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;

        this->encoder.EncoderMode = _encmode;

        this->encoder.IC1Filter = 0x0F;
        this->encoder.IC1Polarity = TIM_INPUTCHANNELPOLARITY_RISING;
        this->encoder.IC1Prescaler = TIM_ICPSC_DIV4;
        this->encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;

        this->encoder.IC2Filter = 0x0F;
        this->encoder.IC2Polarity = TIM_INPUTCHANNELPOLARITY_FALLING;
        this->encoder.IC2Prescaler = TIM_ICPSC_DIV4;
        this->encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;

        if (HAL_TIM_Encoder_Init(&this->timer, &this->encoder) != HAL_OK) {
            printf("Couldn't Init Encoder\r\n");
            while (1) {}
        }

        if(HAL_TIM_Encoder_Start(&this->timer, TIM_CHANNEL_1) != HAL_OK) {
            printf("Couldn't Start Encoder\r\n");
            while (1) {}
        }

        // Update (aka over- and underflow) interrupt enabled
        this->TIM->DIER |= 0x0001;
        // The initialisation process generates an update interrupt, so we'll have to clear the update flag before anything else
        this->TIM->SR &= 0xfffe;
    }

    int32_t encoderHAL::getPulses(bool reset)
    {
        int16_t count = this->TIM->CNT;
        if(reset) {
            switch((uint32_t)this->TIM){
                case TIM1_BASE :
                    TIM1->CNT = 0;
                    break;
                case TIM2_BASE :
                    TIM2->CNT = 0;
                    break;
                case TIM3_BASE :
                    TIM3->CNT = 0;
                    break;
                case TIM4_BASE :
                    TIM4->CNT = 0;
                    break;
                case TIM5_BASE :
                    TIM5->CNT = 0;
                    break;
                case TIM8_BASE :
                    TIM8->CNT = 0;
                    break;
            }
        }
        else {
            switch((uint32_t)this->TIM)
            {
                case TIM1_BASE :
                    return (int32_t)count;
                case TIM2_BASE :
                    return (int32_t)count;
                case TIM3_BASE :
                    return (int32_t)count;
                case TIM4_BASE :
                    return (int32_t)count;
                case TIM5_BASE :
                    return (int32_t)count;
                case TIM8_BASE :
                    return (int32_t)count;
            }
        }

        return (int32_t)count;
    }
    
    TIM_HandleTypeDef* encoderHAL::GetTimer()
    {
        return &this->timer;    
    }

    int encoderHAL::getDistance(bool reset)
    {
        // 4000 is pulse per revolutions
        int count = (this->getPulses(reset)*PI*D_RODA/4000);

        return count;
    }
}