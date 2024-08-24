/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../../KRAI_library/Motor/Motor.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_library/JoystickPS3/JoystickPS3.h"
#include "../../../KRAI_library/JoystickPS3/MappingJoystick.h"

#define PWM BMV1_PWM_MOTOR_1
#define FOR BMV1_FOR_MOTOR_1
#define REV BMV1_REV_MOTOR_1

#define CHA BMV1_ENCODER_1_A
#define CHB BMV1_ENCODER_1_B

#define PPR 537.6

// Object Motor - Encoder - Joystick
Motor motor(PWM, FOR , REV);
encoderKRAI enc(CHA , CHB , PPR , Encoding::X4_ENCODING);



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

//==============================SETUP CANBUS==========================================
#define CAN_TX PD_0
#define CAN_RX PD_1
#define ID_BM_BASE 1
#define ID_BM_GRIPPER 2



int data_timer = 0;
int can_timeout_timer = 0;
CAN can(PA_11, PA_12, 500000); //Cek lagi pin Transmit Receive di Board 1




BMAktuatorKRAI sender (ID_BM_BASE, &millis);
BMAktuatorKRAI gripper (ID_BM_GRIPPER, &millis);

 
//============================BISMILLAH NO GETER GETER===========================//
#define DELTA_MAX 0.3  // Adjust this value as needed

// Variables to store the previous motor speeds
float prevVx = 0;
float prevVy = 0;
float prevOmega = 0;

// Function to limit the rate of change
float limitRateOfChange(float currentValue, float targetValue, float deltaMax) {
    float delta = targetValue - currentValue;

    if (delta > deltaMax) {
        currentValue += deltaMax;
    } else if (delta < -deltaMax) {
        currentValue -= deltaMax;
    } else {
        currentValue = targetValue;
    }

    return currentValue;
}





// Definisikan Lamanya Menerima dan kirim data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
 
//-------------------------UART PS3-------------------------------------------
JoystickPS3 ps3 (BMV1_UART_TX,BMV1_UART_RX);
class TimeOutPS3
{
private:
    uint32_t prevTime = 0;
    uint32_t tempIntegral = 0;
    uint32_t durasiIgnore = 200; // ms
public:
    TimeOutPS3() = default;
    TimeOutPS3(uint32_t durasiIgn) : durasiIgnore(durasiIgn) {}
    void updateTime(uint32_t time) { this->prevTime = time; }
    bool checkTimeOut(bool run, uint32_t timeNow) {
        if (run == false) {
            this->tempIntegral = 0;
            this->prevTime = timeNow;
            return false;
        }
        else {
            this->tempIntegral +=  (timeNow - this->prevTime);
            if (this->tempIntegral > this->durasiIgnore) {
                return true;
            }
            return false;
        }
    }
};

//INISIASI TIMEOUT//
TimeOutPS3 timeOut_R1(200);
TimeOutPS3 timeOut_L1(200);
TimeOutPS3 timeOut_Atas(200);
TimeOutPS3 timeOut_Bawah(200);
TimeOutPS3 timeOut_Kanan(200);
TimeOutPS3 timeOut_Kiri(200);
TimeOutPS3 timeOut_Silang(200);
TimeOutPS3 timeOut_Lingkaran(200);
TimeOutPS3 timeOut_Kotak(200);



float DeltaMAX  =0.3;


float map(float in_min, float in_max, float out_min, float out_max, float input)
{
    return (input - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


//==============================================MAIN PROGRAM======================================//
int main()
{
    //=========================INITIALIZATION====================================
    
    sender.IsAlwaysSend(1);

    ms_tick.attach_us(onMillisecondTicker,1000);
    ps3.setup();

    float RawAxisL_x, RawAxisL_y;
    float RawAxisR_x;
    float vx;
    float vy;
    float omega;


    int timedelay = 500;
    int lastmillis = millis;

    //------------------//
    bool Isduck = false;
    int Duckspd_max = 64;
    int Duckspd_min = -64;
    int DuckOmega_max = 32;
    int DuckOmega_min = -32;
    bool ToGrip =  false;
    int TEST = 0;
    bool isDebugPS3 = false;

    // ---------------- STORAGE -------------------
    bool release_red_ball = false;
    bool release_blue_ball = false;
    bool separator_extended = false;


    // --------------------------------------------------------------------------

    int PS3_OFFSET = 20;
    int SPEED_MS_XY = 2;
    int SPEED_LIMIT_SLOW = 1;
    int SPEED_LIMIT_FAST = 2;
    int SPEED_MS_OMEGA = 2;
    

    while (true)
    {
        ps3.updateMillis(millis);
        ps3.baca_data();
        ps3.olah_data();
        ToGrip = false;
        Isduck = false;
     

       //==============================INPUT GETTER=========================================/
        
        //==========================================INPUT NO  JITTER==========================//
        isDebugPS3 = true;
        if (isDebugPS3)
        {
            //printf("RX : %d RY : %d LX : %d LY : %d\n", ps3.getRX(), ps3.getRY(), ps3.getLX(), ps3.getLY());
            // printf("R1 : %d R2 : %d L1 : %d L2 : %d\n", ps3.getR1(), ps3.getR2(), ps3.getL1(), ps3.getL2());
            printf("Kotak : %d Silang : %d Lingkaran : %d Segitiga : %d\n", ps3.getKotak(), ps3.getSilang(), ps3.getLingkaran(), ps3.getSegitiga());
            //printf("Up : %d Left : %d Right : %d Down : %d\n", ps3.getButtonUp(), ps3.getButtonLeft(), ps3.getButtonRight(), ps3.getButtonDown());
            // printf("Start : %d Select : %d\n", ps3.getStart(), ps3.getSelect());
        }

        //==========================================UNTUK PERGERAKAN BASE=================================//
       
       //Reset Value
        vx = 0;
        vy = 0;
        omega = 0;
        release_blue_ball = false;
        release_red_ball = false;
        separator_extended = false;
        
        float deltaVRight, deltaVLeft;
        float deltaVFor, deltaVRev;
        
        if (ps3.getSilang())
        {
            if (timeOut_Silang.checkTimeOut(true, millis))
            {
                release_blue_ball = true;
            }
        }
        else {
            timeOut_Silang.checkTimeOut(false, millis);
        }

        if (ps3.getLingkaran())
        {
            if (timeOut_Lingkaran.checkTimeOut(true, millis))
            {
                release_red_ball = true;
            }
        }
        else {
            timeOut_Lingkaran.checkTimeOut(false, millis);
        }

        if (ps3.getKotak())
        {
            if (timeOut_Kotak.checkTimeOut(true, millis))
            {
                separator_extended = true;
            }
        }
        else {
            timeOut_Kotak.checkTimeOut(false, millis);
        }

        if (ps3.getL1())
        {
            if (timeOut_L1.checkTimeOut(true, millis))
            {
                omega += 0.7;
            }
        }
        else {
            timeOut_L1.checkTimeOut(false, millis);
        }

        if (ps3.getR1())
        {
            if (timeOut_R1.checkTimeOut(true, millis))
            {
                omega += -0.7;
            }
        }
        else {
            timeOut_R1.checkTimeOut(false, millis);
        }
        
        if (ps3.getButtonUp())
        {
            if (timeOut_Atas.checkTimeOut(true, millis))
            {
                vy += 1;
            }
        }
        else {
            timeOut_Atas.checkTimeOut(false, millis);
        }

        if (ps3.getButtonDown())
        {
            if (timeOut_Bawah.checkTimeOut(true, millis))
            {
                vy += -1;
            }
        }
        else {
            timeOut_Bawah.checkTimeOut(false, millis);
        }

        if (ps3.getButtonLeft())
        {
            if (timeOut_Kiri.checkTimeOut(true, millis))
            {
                vx += -1;
            }
        }
        else {
            timeOut_Kiri.checkTimeOut(false, millis);
        }

        if (ps3.getButtonRight())
        {
            if (timeOut_Kanan.checkTimeOut(true, millis))
            {
                vx += 1;
            }
        }
        else {
            timeOut_Kanan.checkTimeOut(false, millis);
        }
        
        // Set the motor speeds
        #define SPEED_MULTIPLIER 1.5f
        #define OMEGA_MULTIPLIER 1.5f

        sender.setMotor1(int(vx*1000*SPEED_MULTIPLIER));
        sender.setMotor2(int(vy*1000*SPEED_MULTIPLIER));
        sender.setInteger(int(omega*1000*OMEGA_MULTIPLIER));
        sender.setSwitch1(release_blue_ball);
        sender.setSwitch2(release_red_ball);
        sender.setSwitch3(separator_extended);

         //=============================CAN BUS COMM==========================//
    
        sender.sendCAN(sender.getNoBM(),5);
        gripper.sendCAN(gripper.getNoBM(),5);


    }
    return 0;
}