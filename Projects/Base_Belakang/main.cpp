/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

// library
#include "mbed.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../../KRAI_library/InverseKinematics/Omni4Wheel.h"
#include "../../../KRAI_library/Motor/Motor.h"
#include  "../../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../../KRAI_library/MovingAverage/MovingAverage.h"

// define motor 1
#define PWM1 BMV1_PWM_MOTOR_1
#define FOR1 BMV1_FOR_MOTOR_1
#define REV1 BMV1_REV_MOTOR_1

#define CHA1 BMV1_ENCODER_1_A
#define CHB1 BMV1_ENCODER_1_B

// define motor 2
#define PWM2 BMV1_PWM_MOTOR_2
#define FOR2 BMV1_FOR_MOTOR_2
#define REV2 BMV1_REV_MOTOR_2

#define CHA2 BMV1_ENCODER_2_A
#define CHB2 BMV1_ENCODER_2_B

// define encoder PPR
// PPR = Pulses Per Revolution
#define PPR 537.6f //537.6

// define diameter and distance from center
#define DfromCenter 0.35f

// diameter roda
#define Diameter 0.15f

DigitalOut led(PC_13);

// =================SETUP MOTOR / CREATE OBJECT MOTOR===============================
//Motor_1 belakang kanan (BR)
Motor motor_1(PWM1, FOR1, REV1);
encoderKRAI enc_motor_1(CHA1, CHB1, PPR, Encoding::X4_ENCODING);

//Motor_2 belakang kiri (BL)
Motor motor_2(PWM2, FOR2, REV2);
encoderKRAI enc_motor_2(CHA2, CHB2, PPR, Encoding::X4_ENCODING);
MovingAverage movAvg(10);

//=========================SETUP UART SERIAL PRINT===================================
// untuk serial print doang
#define SERIAL_TX PB_6
#define SERIAL_RX PB_7
#define SERIAL_BAUDRATE 115200

static BufferedSerial serial_port(SERIAL_TX, SERIAL_RX, SERIAL_BAUDRATE);
FileHandle *mbed::mbed_override_console(int fd) {
    return &serial_port;
}
//-----------------------------------------------------------------------------------

//============================SETUP BASIC TIMER======================================
// timer pake variabel millis
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

// setup board manager; setup object can
CAN can(PA_11, PA_12, 500000);
BMAktuatorKRAI BM_Belakang(ID_BM_PENERIMA, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // ms
#define TS_SEND_CAN     5   // ms
//-----------------------------------------------------------------------------------

//=======================WALAHI INVERSE KINEMATICS KERJA PLEASE=========================
// untuk define object omniwheel

Omni4Wheel Omnibase(&millis, DfromCenter, Diameter, 0.0f);

// define variabel untuk kecepatan; refrensi posisi dari robot
float Vx, Vy, Omega;
float BR_speed, BL_speed;

#define CONSTSPEED 1.0f
#define CONSTOMEGA 0.5f
#define ANALOG_SCALE_MOVE 256.0f
#define ANALOG_SCALE_ROTATE 64.0f

// =====================================================================================

int main()
{

    //=========================INITIALIZATION====================================
    // millis update terus
    ms_tick.attach_us(onMillisecondTicker,1000);

    // board manager kirim terus updatenya
    // BM_Belakang.IsAlwaysSend(1);

    // --------------------------------------------------------------------------
    
    float pulseThen_BL = enc_motor_1.getPulses();
    float pulseThen_BR = enc_motor_2.getPulses();
    float lastmillispulse = millis;
    float rotatePerSec_BL;
    float rotatePerSec_BR;
    float PWM_motor_BL;
    float PWM_motor_BR;


    while (true)
    {
        // if read can, dia akan set speed untuk motor 1 dan motor 2
        // yang dalam kurung itu dalam milisecond
        

        if (BM_Belakang.readCAN(3)){
            if (millis - data_timer > 500)
            {
                led = !led;
                data_timer = millis;
            }
            can_timeout_timer = millis;
        }

        if (millis - lastmillispulse > 10.0f){
            rotatePerSec_BL = (enc_motor_1.getPulses() - pulseThen_BL)/(PPR*0.01);
            rotatePerSec_BR = (enc_motor_2.getPulses() - pulseThen_BR)/(PPR*0.01);

            rotatePerSec_BL = movAvg.movingAverage(rotatePerSec_BL);
            rotatePerSec_BR = movAvg.movingAverage(rotatePerSec_BR);

            pulseThen_BL = enc_motor_1.getPulses();
            pulseThen_BR = enc_motor_2.getPulses();

            lastmillispulse = millis;    
        }
        //BM motor 1 = Kiri Kanan
        //BM motor 2 = Maju Mundur
        //integer = Omega
        
        // menerima data RC dari CAN untuk diubah jadi data kecepatan m/s
        Vx = (static_cast<float>(BM_Belakang.getMotor1()) / ANALOG_SCALE_MOVE)*CONSTSPEED;
        Vy = -(static_cast<float>(BM_Belakang.getMotor2()) / ANALOG_SCALE_MOVE)*CONSTSPEED;
        Omega = (static_cast<float>(BM_Belakang.getInteger()) / ANALOG_SCALE_ROTATE)*CONSTOMEGA;
        
        // set vx, vy, omega untuk ke omnibase
        Omnibase.setVx(Vx);
        Omnibase.setVy(Vy);
        Omnibase.setOmega(Omega);

        // calculate inverse kinematics untuk dapat speed setiap motor
        Omnibase.InverseCalc();

        BL_speed = Omnibase.getBLSpeed();
        BR_speed = Omnibase.getBRSpeed();

        // untuk serial print doang
        printf("Vx = %f, Vy = %f, Omega = %f, BR_speed = %f, BL_speed = %f, BR_RPS = %f, BL_RPS = %f\n", Vx, Vy, Omega, BR_speed, BL_speed, rotatePerSec_BR, rotatePerSec_BL);

        // set speed motor 1 dan motor 2 sesuai dengan hasil inverse kinematics
        motor_1.speed(Omnibase.getBRSpeed());
        motor_2.speed(Omnibase.getBLSpeed());
    }
    

    printf("Hello, Mbed!\n");
    return 0;
}