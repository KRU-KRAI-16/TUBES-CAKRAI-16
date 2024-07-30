/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../../KRAI_library/InverseKinematics/Omni4Wheel.h"
#include "../../../KRAI_library/Motor/Motor.h"
#include  "../../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"

#define PWM1 BMV1_PWM_MOTOR_1
#define FOR1 BMV1_FOR_MOTOR_1
#define REV1 BMV1_REV_MOTOR_1

#define CHA1 BMV1_ENCODER_1_A
#define CHB1 BMV1_ENCODER_1_B

#define PWM2 BMV1_PWM_MOTOR_2
#define FOR2 BMV1_FOR_MOTOR_2
#define REV2 BMV1_REV_MOTOR_2

#define CHA2 BMV1_ENCODER_2_A
#define CHB2 BMV1_ENCODER_2_B

#define PPR 7*4*19.2 //537.6

#define DfromCenter 35.0f

#define Diameter 15.0f

Motor motor_1(PWM1, FOR1, REV1);
encoderKRAI enc_motor_1(CHA1, CHB1, PPR, Encoding::X4_ENCODING);

Motor motor_2(PWM2, FOR2, REV2);
encoderKRAI enc_motor_2(CHA2, CHB2, PPR, Encoding::X4_ENCODING);

//=========================SETUP UART SERIAL PRINT===================================
#define SERIAL_TX PB_6
#define SERIAL_RX PB_7
#define SERIAL_BAUDRATE 115200

static BufferedSerial serial_port(SERIAL_TX, SERIAL_RX, SERIAL_BAUDRATE);
FileHandle *mbed::mbed_override_console(int fd) {
    return &serial_port;
}
//-----------------------------------------------------------------------------------

//============================SETUP BASIC TIMER======================================
Ticker ms_tick;
uint32_t millis = 0;
void onMillisecondTicker(void)
{
    // this code will run every millisecond
    millis++;
    
}

// SETUP TIMER IN US_TICKER_READ
// US = µ Second -> Timer dengan satuan detik 10^6 (µ)
int ms_ticker_read(){

    return us_ticker_read()/1000;

}
//-----------------------------------------------------------------------------------

//==============================SETUP CANBUS==========================================
#define CAN_TX CAN_TX_Bluepill
#define CAN_RX CAN_RX_Bluepill

#define ID_BM_PENERIMA 1

int data_timer = 0;
int can_timeout_timer = 0;

CAN can(PA_11, PA_12, 500000);
BMAktuatorKRAI BM_Belakang(ID_BM_PENERIMA, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // ms
#define TS_SEND_CAN     5   // ms
//-----------------------------------------------------------------------------------

Omni4Wheel base(&millis, DfromCenter, Diameter, 0.0f);

int main()
{

    //=========================INITIALIZATION====================================

    ms_tick.attach_us(onMillisecondTicker,1000);
    BM_Belakang.IsAlwaysSend(1);

    // --------------------------------------------------------------------------

    if (BM_Belakang.readCAN(500)){
        
        //motor 1 = Maju mundur
        //motor 2 = Kiri Kanan
        //integer = Omega       

    }


    while (1)
    {
        /* code */
    }
    

    printf("Hello, Mbed!\n");
    return 0;
}