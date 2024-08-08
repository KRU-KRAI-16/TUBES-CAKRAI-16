/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../../KRAI_library/Motor/Motor.h"
#include "../../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_library/InverseKinematics/Omni4Wheel.h"

#define PWM_FL BMV1_PWM_MOTOR_1
#define FOR_FL BMV1_FOR_MOTOR_1
#define REV_FL BMV1_REV_MOTOR_1

#define PWM_FR BMV1_PWM_MOTOR_2
#define FOR_FR BMV1_FOR_MOTOR_2
#define REV_FR BMV1_REV_MOTOR_2

#define CHA_FL BMV1_ENCODER_1_A
#define CHB_FL BMV1_ENCODER_1_B

#define CHA_FR BMV1_ENCODER_2_A
#define CHB_FR BMV1_ENCODER_2_B

#define PPR 537.6

// Object Motor - Encoder
Motor motor_FL(PWM_FL, FOR_FL , REV_FL);
Motor motor_FR(PWM_FR, FOR_FR , REV_FR);
encoderKRAI encoder_FL(CHA_FL , CHB_FL , PPR , Encoding::X4_ENCODING);
encoderKRAI encoder_FR(CHA_FR , CHB_FR , PPR , Encoding::X4_ENCODING);

//============================SETUP BASIC TIMER======================================
Ticker ms_tick;
uint32_t millis = 0;
void onMillisecondTicker(void)
{
    // this code will run every millisecond
    millis++;
}

DigitalOut led(PC_13);

//-----------------------------------------------------------------------------------

//=========================SETUP UART SERIAL PRINT===================================
#define SERIAL_TX PB_6
#define SERIAL_RX PB_7
#define SERIAL_BAUDRATE 115200

static BufferedSerial serial_port(SERIAL_TX, SERIAL_RX, SERIAL_BAUDRATE);
FileHandle *mbed::mbed_override_console(int fd) {
    return &serial_port;
}
//-----------------------------------------------------------------------------------

//============================== SETUP CANBUS ==========================================

#define CAN_TX PA_11
#define CAN_RX PA_12
#define ID_BM_BASE 1
int data_timer = 0;
int can_timeout_timer = 0;
CAN can(CAN_TX, CAN_RX, 500000);

/*
getMotor1() -> vx
getMotor2() -> vy
getInteger() -> omega
*/
BMAktuatorKRAI BM_Base(ID_BM_BASE, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
//-----------------------------------------------------------------------------------

//============================== SETUP INVERSE KINEMATICS =============================

#define SPEED_CONST 1.0f
#define OMEGA_CONST 0.5f
#define ANALOG_SCALE_MOVE 256.0f
#define ANALOG_SCALE_ROTATE 128.0f
unsigned long* millisPtr;
Omni4Wheel omni4Wheel(millisPtr, 0.3, 0.05);

//-------------------------------------------------------------------------------------

int main()
{
    //=========================INITIALIZATION====================================

    ms_tick.attach_us(onMillisecondTicker,1000);
    // BM_Base.IsAlwaysSend(1);

    // --------------------------------------------------------------------------

    float vx, vy, omega;
    float FL_speed, FR_speed;

    while (true)
    {

        if(BM_Base.readCAN(TS_READ_CAN)){
            if (millis - data_timer > 500)
            {
                led = !led;
                data_timer = millis;
            }


            can_timeout_timer = millis;
        }


        //============================USER CODE==================================

        vx = (static_cast<float>(BM_Base.getMotor1())/ANALOG_SCALE_MOVE) * SPEED_CONST;
        vy = (static_cast<float>(BM_Base.getMotor2())/ANALOG_SCALE_MOVE) * SPEED_CONST;
        omega = (static_cast<float>(BM_Base.getInteger())/ANALOG_SCALE_ROTATE) * OMEGA_CONST;
        
        omni4Wheel.setVx(vx);
        omni4Wheel.setVy(vy);
        omni4Wheel.setOmega(omega);

        omni4Wheel.InverseCalc();

        FL_speed = omni4Wheel.getFLSpeed();
        FR_speed = omni4Wheel.getFRSpeed();

        printf("vx: %f, vy: %f, omega: %f, FL_speed = %f, FR_speed = %f\n", vx, vy, omega, FL_speed, FR_speed);

        motor_FL.speed(FL_speed);
        motor_FR.speed(FR_speed);

        // ----------------------------------------------------------------------
    }
    
    return 0;
}