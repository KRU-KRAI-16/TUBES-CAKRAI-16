/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

// ========================LIBRARIES==========================
// BOARD MANAGER
#include "../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"

// MOTOR AND ENCODER
#include "../../KRAI_library/Motor/Motor.h"
#include "../../KRAI_library/encoderKRAI/encoderKRAI.h"

// TEMP PID
#include "../../KRAI_library/MiniPID/MiniPID.h"
#include "../../KRAI_library/MovingAverage/MovingAverage.h"

// ADRC V2
#include "../../KRAI_library/ADRC_V2/ADRC_V2.h"

//--------------------------------------------------------------

// DECLARATION MOTORS
// BR = MOTOR 1
// FR = MOTOR 2

#define PWM_BR BMV1_PWM_MOTOR_1
#define FOR_BR BMV1_FOR_MOTOR_1
#define REV_BR BMV1_REV_MOTOR_1

#define PWM_FR BMV1_PWM_MOTOR_2
#define FOR_FR BMV1_FOR_MOTOR_2
#define REV_FR BMV1_REV_MOTOR_2

#define CHA_BR BMV1_ENCODER_1_A
#define CHB_BR BMV1_ENCODER_1_B

#define CHA_FR BMV1_ENCODER_2_A
#define CHB_FR BMV1_ENCODER_2_B

#define PPR 537.6f

// Object Motor - Encoder
Motor motor_BR(PWM_BR, FOR_BR , REV_BR);
Motor motor_FR(PWM_FR, FOR_FR , REV_FR);

encoderKRAI encoder_BR(CHB_BR , CHA_BR , PPR , Encoding::X4_ENCODING);
encoderKRAI encoder_FR(CHB_FR , CHA_FR , PPR , Encoding::X4_ENCODING);

MovingAverage movAvgBR(10);
MovingAverage movAvgFR(10);

// PID SEMENTARA
// PID params
// double Kp = 0.10f;
// double Ki = 0.0034f;
// double Kd = 0.04f;

// PIDs
// MiniPID pid_BR(Kp, Ki, Kd);
// MiniPID pid_FR(Kp, Ki, Kd);

// ADRC V2
// Parameter for normal speed
uint32_t timeSampling = 7;

float b0_FrontMotor_normal = 430;
float b0_BackMotor_normal = 430;
float tSettle_normal = 0.5;
float zESO_normal = 7.25;
float incrementInt_normal = 0.09;

ADRC_V2 ADRC_BR(timeSampling / 1000.0f, b0_FrontMotor_normal, tSettle_normal, zESO_normal, incrementInt_normal);
ADRC_V2 ADRC_FR(timeSampling / 1000.0f, b0_BackMotor_normal, tSettle_normal, zESO_normal, incrementInt_normal);


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

DigitalOut led(PC_13);

//=========================SETUP UART SERIAL PRINT===================================
#define SERIAL_TX PB_6
#define SERIAL_RX PB_7
#define SERIAL_BAUDRATE 115200

static BufferedSerial serial_port(SERIAL_TX, SERIAL_RX, SERIAL_BAUDRATE);
FileHandle *mbed::mbed_override_console(int fd) {
    return &serial_port;
}
//-----------------------------------------------------------------------------------

//==============================SETUP CANBUS==========================================
#define CAN_TX PA_11
#define CAN_RX PA_12
#define ID_BM_BASE_KANAN 1
int data_timer = 0;
int can_timeout_timer = 0;
CAN can(CAN_TX, CAN_RX, 500000);

/* 
getMotor1() -> BR
gedtMotor2() -> FR
*/

BMAktuatorKRAI BM_Base(ID_BM_BASE_KANAN, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
//-----------------------------------------------------------------------------------
#define ANALOG_SCALE 1000.0f

int main()
{
    //=========================INITIALIZATION====================================

    ms_tick.attach_us(onMillisecondTicker,1000);

    float BR_setvalue, FR_setvalue;

    // PID SEMENTARA
    //float BR_speed, FR_speed;

    float pulseThen_BR = encoder_BR.getPulses();
    float pulseThen_FR = encoder_FR.getPulses();
    float lastmillispulse = millis;
    float rotatePerSec_BR;
    float rotatePerSec_FR;
    float PWM_motor_BR;
    float PWM_motor_FR;


    // pid_BR.setOutputLimits(-1,1);
    // pid_FR.setOutputLimits(-1,1);
    
    // --------------------------------------------------------------------------

    while (true)
    {
        //============================USER CODE==================================

        if(BM_Base.readCAN(TS_READ_CAN)){
            if (millis - data_timer > 500)
            {
                led = !led;
                data_timer = millis;
            }

            can_timeout_timer = millis;
        }

        BR_setvalue = BM_Base.getMotor1();
        BR_setvalue = BR_setvalue / ANALOG_SCALE;
        FR_setvalue = BM_Base.getMotor2();
        FR_setvalue = FR_setvalue / ANALOG_SCALE;

        float explicit_r0 = 0.0f;

        // if (BR_setvalue == 0.0f  && FR_setvalue == 0.0f ){
        //     explicit_r0 = 0.0f;
        // }

        // printf("mot1 : %f, mot2 : %f \n", BR_setvalue, FR_setvalue);

        if (millis - lastmillispulse > timeSampling){
            lastmillispulse = millis;
            rotatePerSec_BR = (encoder_BR.getPulses() - pulseThen_BR)/(PPR*0.01);
            rotatePerSec_FR = (encoder_FR.getPulses() - pulseThen_FR)/(PPR*0.01);

            rotatePerSec_BR = movAvgBR.movingAverage(rotatePerSec_BR);
            rotatePerSec_FR = movAvgFR.movingAverage(rotatePerSec_FR);

            pulseThen_BR = encoder_BR.getPulses();
            pulseThen_FR = encoder_FR.getPulses();
            
            PWM_motor_BR = ADRC_BR.createInputSignal(ADRC_BR.fhan_setPointTrajectory(BR_setvalue, explicit_r0), rotatePerSec_BR, 1.0f);
            PWM_motor_FR = ADRC_FR.createInputSignal(ADRC_FR.fhan_setPointTrajectory(FR_setvalue, explicit_r0), rotatePerSec_FR, 1.0f);

            // PID SEMENTARA
            // PWM_motor_BR = pid_BR.getOutput(rotatePerSec_BR, BR_setvalue);
            // PWM_motor_FR = pid_FR.getOutput(rotatePerSec_FR, FR_setvalue);
        }

        // ----------------------------------------------------------------------
        motor_BR.speed(PWM_motor_BR);
        motor_FR.speed(PWM_motor_FR);
    }
    
    
    return 0;
}