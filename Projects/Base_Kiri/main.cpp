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
// BL = MOTOR 1
// FL = MOTOR 2

#define PWM_BL BMV1_PWM_MOTOR_1
#define FOR_BL BMV1_FOR_MOTOR_1
#define REV_BL BMV1_REV_MOTOR_1

#define PWM_FL BMV1_PWM_MOTOR_2
#define FOR_FL BMV1_FOR_MOTOR_2
#define REV_FL BMV1_REV_MOTOR_2

#define CHA_BL BMV1_ENCODER_1_A
#define CHB_BL BMV1_ENCODER_1_B

#define CHA_FL BMV1_ENCODER_2_A
#define CHB_FL BMV1_ENCODER_2_B

#define PPR 537.6f

// Object Motor - Encoder
Motor motor_BL(PWM_BL, FOR_BL , REV_BL);
Motor motor_FL(PWM_FL, FOR_FL , REV_FL);

encoderKRAI encoder_BL(CHB_BL , CHA_BL , PPR , Encoding::X4_ENCODING);
encoderKRAI encoder_FL(CHB_FL , CHA_FL , PPR , Encoding::X4_ENCODING);

MovingAverage movAvgBL(10);
MovingAverage movAvgFL(10);

// PID SEMENTARA
// PID params
// double Kp = 0.10f;
// double Ki = 0.0034f;
// double Kd = 0.04f;

// PIDs
// MiniPID pid_BL(Kp, Ki, Kd);
// MiniPID pid_FL(Kp, Ki, Kd);

// ADRC V2
// Parameter for normal speed
uint32_t timeSampling = 7;

float b0_FrontMotor_normal = 430;
float b0_BackMotor_normal = 430;
float tSettle_normal = 0.5;
float zESO_normal = 7.25;
float incrementInt_normal = 0.09;

ADRC_V2 ADRC_BL(timeSampling / 1000.0f, b0_FrontMotor_normal, tSettle_normal, zESO_normal, incrementInt_normal);
ADRC_V2 ADRC_FL(timeSampling / 1000.0f, b0_BackMotor_normal, tSettle_normal, zESO_normal, incrementInt_normal);


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
#define ID_BM_BASE_KIRI 4
int data_timer = 0;
int can_timeout_timer = 0;
CAN can(CAN_TX, CAN_RX, 500000);

/* 
getMotor1() -> BL
gedtMotor2() -> FL
*/

BMAktuatorKRAI BM_Base(ID_BM_BASE_KIRI, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
//-----------------------------------------------------------------------------------

#define ANALOG_SCALE 1000.0f

int main()
{
    //=========================INITIALIZATION====================================

    ms_tick.attach_us(onMillisecondTicker,1000);

    float BL_setvalue, FL_setvalue;

    // PID SEMENTARA
    //float BL_speed, FL_speed;

    float pulseThen_BL = encoder_BL.getPulses();
    float pulseThen_FL = encoder_FL.getPulses();
    float lastmillispulse = millis;
    float rotatePerSec_BL;
    float rotatePerSec_FL;
    float PWM_motor_BL;
    float PWM_motor_FL;

    // pid_BL.setOutputLimits(-1,1);
    // pid_FL.setOutputLimits(-1,1);
    
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

        BL_setvalue = BM_Base.getMotor1();
        BL_setvalue = BL_setvalue / ANALOG_SCALE;
        FL_setvalue = BM_Base.getMotor2();
        FL_setvalue = FL_setvalue / ANALOG_SCALE;

        // printf("mot1 : %f, mot2 : %f \n", BL_setvalue, FL_setvalue);

        float explicit_r0 = 0.0f;

        // if (BL_setvalue == 0.0f  && FL_setvalue == 0.0f ){
        //     explicit_r0 = 0.0f;
        // }

        // PID SEMENTARA
        if (millis - lastmillispulse > timeSampling){
            lastmillispulse = millis;
            rotatePerSec_BL = (encoder_BL.getPulses() - pulseThen_BL)/(PPR*0.01);
            rotatePerSec_FL = (encoder_FL.getPulses() - pulseThen_FL)/(PPR*0.01);

            rotatePerSec_BL = movAvgBL.movingAverage(rotatePerSec_BL);
            rotatePerSec_FL = movAvgFL.movingAverage(rotatePerSec_FL);

            pulseThen_BL = encoder_BL.getPulses();
            pulseThen_FL = encoder_FL.getPulses();

            PWM_motor_BL = ADRC_BL.createInputSignal(ADRC_BL.fhan_setPointTrajectory(BL_setvalue, explicit_r0), rotatePerSec_BL, 1.0f);
            PWM_motor_FL = ADRC_FL.createInputSignal(ADRC_FL.fhan_setPointTrajectory(FL_setvalue, explicit_r0), rotatePerSec_FL, 1.0f);

            
            // PWM_motor_BL = pid_BL.getOutput(rotatePerSec_BL, BL_setvalue);
            // PWM_motor_FL = pid_FL.getOutput(rotatePerSec_FL, FL_setvalue);
        }

        // ----------------------------------------------------------------------
        motor_BL.speed(PWM_motor_BL);
        motor_FL.speed(PWM_motor_FL);
    }
    
    
    return 0;
}