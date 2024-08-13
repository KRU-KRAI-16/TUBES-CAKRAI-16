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
#include "../../../KRAI_library/MiniPID/MiniPID.h"
#include "../../../KRAI_library/MovingAverage/MovingAverage.h"

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

#define PPR 537.6f

// Object Motor - Encoder
Motor motor_FL(PWM_FL, FOR_FL , REV_FL);
Motor motor_FR(PWM_FR, FOR_FR , REV_FR);

encoderKRAI encoder_FL(CHA_FL , CHB_FL , PPR , Encoding::X4_ENCODING);
encoderKRAI encoder_FR(CHA_FR , CHB_FR , PPR , Encoding::X4_ENCODING);

MovingAverage movAvg(10);

//PID params
double Kp = 0.10f;
double Ki = 0.0034f;
double Kd = 0.04f;

//PIDs
MiniPID pid_FL(Kp, Ki, Kd);
MiniPID pid_FR(Kp, Ki, Kd);

//PID set output limit


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
#define ANALOG_SCALE_ROTATE 64.0f
unsigned long* millisPtr;
Omni4Wheel omni4Wheel(millisPtr, 0.395, 0.15);

//-------------------------------------------------------------------------------------

int main()
{
    //=========================INITIALIZATION====================================

    ms_tick.attach_us(onMillisecondTicker,1000);
    // BM_Base.IsAlwaysSend(1);

    // --------------------------------------------------------------------------

    //inverse kinematics
    float vx, vy, omega;
    float FL_speed, FR_speed;
    float FL_rps, FR_rps;

    //hitung kecepatan masing - masing roda
    float pulseThen_FL = encoder_FL.getPulses();
    float pulseThen_FR = encoder_FR.getPulses();
    float lastmillispulse = millis;
    float rotatePerSec_FL;
    float rotatePerSec_FR;
    float PWM_motor_FL;
    float PWM_motor_FR;
    
    //PID set limit
    pid_FL.setOutputLimits(-1,1);
    pid_FR.setOutputLimits(-1,1);
    

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
        vx = (static_cast<float>(BM_Base.getMotor1())/ANALOG_SCALE_MOVE) * SPEED_CONST;
        vy = -(static_cast<float>(BM_Base.getMotor2())/ANALOG_SCALE_MOVE) * SPEED_CONST;
        omega = -(static_cast<float>(BM_Base.getInteger())/ANALOG_SCALE_ROTATE) * OMEGA_CONST;
        
        omni4Wheel.setVx(vx);
        omni4Wheel.setVy(vy);
        omni4Wheel.setOmega(omega);

        omni4Wheel.InverseCalc();

        // FL_speed = omni4Wheel.getFLSpeed();
        // FR_speed = omni4Wheel.getFRSpeed();

        FL_rps = omni4Wheel.getFLSpeedRPS();
        FR_rps = omni4Wheel.getFRSpeedRPS();

        if (millis - lastmillispulse > 10.0f){
            rotatePerSec_FL = (encoder_FL.getPulses() - pulseThen_FL)/(PPR*0.01);
            rotatePerSec_FR = (encoder_FR.getPulses() - pulseThen_FR)/(PPR*0.01);

            rotatePerSec_FL = movAvg.movingAverage(rotatePerSec_FL);
            rotatePerSec_FR = movAvg.movingAverage(rotatePerSec_FR);

            pulseThen_FL = encoder_FL.getPulses();
            pulseThen_FR = encoder_FR.getPulses();
            
            PWM_motor_FL = pid_FL.getOutput(rotatePerSec_FL, FL_rps);
            PWM_motor_FR = pid_FR.getOutput(rotatePerSec_FR, FR_rps);

            lastmillispulse = millis;
        }


        //============================USER CODE==================================

        

        // printf("vx: %f, vy: %f, omega: %f, FL_speed = %f, FR_speed = %f, FL_RPS = %f, FR_RPS = %f\n", vx, vy, omega, FL_speed, FR_speed, rotatePerSec_FL, rotatePerSec_FR);
        // printf("FL_setpoint = %f, FR_setpoint = %f, FL_omega = %f, FR_omega = %f\n", FL_rps, FR_rps, rotatePerSec_FL, rotatePerSec_FR);
        // printf("FL_PWM = %f, FL_setpoint = %f, FL_omega = %f\n", PWM_motor_FL, FL_rps, rotatePerSec_FL);   
        

        motor_FL.speed(PWM_motor_FL);
        motor_FR.speed(PWM_motor_FR);

        // ----------------------------------------------------------------------
    }
    
    return 0;
}