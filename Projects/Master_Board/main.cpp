/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"

// Board Manager
#include "../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"

// Joystick
#include "../../KRAI_library/JoystickPS3/JoystickPS3.h"

// Inverse Kinematics
#include "../../KRAI_library/InverseKinematics/Omni4Wheel.h"

//=========================SETUP BASIC TIMER======================================
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
#define ID_BM_BASE_KANAN 1
#define ID_BM_BASE_KIRI 4
#define ID_BM_GRIPPER 3
#define ID_BM_STORAGE 5

int data_timer = 0;
int can_timeout_timer = 0;
CAN can(PA_11, PA_12, 500000); //Cek lagi pin Transmit Receive di Board 1

BMAktuatorKRAI can_kanan (ID_BM_BASE_KANAN, &millis);
BMAktuatorKRAI can_kiri (ID_BM_BASE_KIRI, &millis);
BMAktuatorKRAI gripper (ID_BM_GRIPPER, &millis);
BMAktuatorKRAI storage (ID_BM_STORAGE, &millis);

// Definisikan Lamanya Menerima dan kirim data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
//-----------------------------------------------------------------------------------

//==============================SETUP INVERSE KINEMATICS=============================

#define SPEED_CONST 2.0f
#define OMEGA_CONST 0.5f
#define ANALOG_SCALE 1000.0f
unsigned long* millisPtr;

Omni4Wheel omniwheel(millisPtr, 0.395, 0.15);

//-----------------------------------------------------------------------------------

//==============================UART PS3=============================================
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
TimeOutPS3 timeOut_R2(200);
TimeOutPS3 timeOut_R1(200);
TimeOutPS3 timeOut_L1(200);
TimeOutPS3 timeOut_Atas(200);
TimeOutPS3 timeOut_Bawah(200);
TimeOutPS3 timeOut_Kanan(200);
TimeOutPS3 timeOut_Kiri(200);
TimeOutPS3 timeOut_Silang(200);
TimeOutPS3 timeOut_Lingkaran(200);
TimeOutPS3 timeOut_Kotak(200);
TimeOutPS3 timeOut_Segitiga(200);
TimeOutPS3 timeOut_L2(200);


int main()
{
    //=========================INITIALIZATION====================================
    can_kanan.IsAlwaysSend(1);
    can_kiri.IsAlwaysSend(1);

    ms_tick.attach_us(onMillisecondTicker,1000);
    
    led = 1;
    
    ps3.setup();

    float vx;
    float vy;
    float omega;

    bool isDebug = false;
    float FL_setvalue, FR_setvalue, BL_setvalue, BR_setvalue;

    float vx_mag = 1;
    float vy_mag = 1;
    float omega_mag = 0.7;
    bool SnailMode = false;

    //Bismillah

    // float deltaVRight, deltaVLeft;
    // float deltaVFor, deltaVRev;

    // --------------------------------------------------------------------------

    // ---------------- STORAGE -------------------
    bool release_red_ball = false;
    bool release_blue_ball = false;
    bool separator_extended = false;
    bool gripper_grip = false;
    bool gripper_lift = false;

    //TOGGLE SPEED MODE
    bool toggleL2 = false;
    //true -> on ; false -> off

    while (true)
    {
        ps3.updateMillis(millis);
        ps3.baca_data();
        ps3.olah_data();

        //============================DEBUG GAMING==================================

        isDebug = false;
        if (isDebug)
        {
            // printf("RX : %d RY : %d LX : %d LY : %d", ps3.getRX(), ps3.getRY(), ps3.getLX(), ps3.getLY());
            // printf("R1 : %d R2 : %d L1 : %d L2 : %d", ps3.getR1(), ps3.getR2(), ps3.getL1(), ps3.getL2());
            // printf("Kotak : %d Silang : %d Lingkaran : %d Segitiga : %d\n", ps3.getKotak(), ps3.getSilang(), ps3.getLingkaran(), ps3.getSegitiga());
            // printf("Up : %d Left : %d Right : %d Down : %d", ps3.getButtonUp(), ps3.getButtonLeft(), ps3.getButtonRight(), ps3.getButtonDown());
            // printf("Start : %d Select : %d\n", ps3.getStart(), ps3.getSelect());
            // printf("vx : %f vy : %f omega : %f ", vx, vy, omega);
            // printf("FL : %f FR : %f BL : %f BR : %f \n", FL_setvalue, FR_setvalue, BL_setvalue, BR_setvalue);

        }
        // --------------------------------------------------------------------------

        //==========================JOYSTICK PS3 MOVEMENT============================
               //Reset Value
        vx = 0;
        vy = 0;
        omega = 0;
        release_blue_ball = false;
        release_red_ball = false;
        separator_extended = false;
        gripper_grip = false;
        gripper_lift = false;

        if (ps3.getR2())
        {
            if (timeOut_R2.checkTimeOut(true, millis))
            {
                gripper_lift = true;
            }
        }
        else {
            timeOut_R2.checkTimeOut(false, millis);
        }

        if (ps3.getSegitiga())
        {
            if (timeOut_Segitiga.checkTimeOut(true, millis))
            {
                gripper_grip = true;
            }
        }
        else {
            timeOut_Segitiga.checkTimeOut(false, millis);
        }

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
                omega += omega_mag;
            }
        }
        else {
            timeOut_L1.checkTimeOut(false, millis);
        }

        if (ps3.getR1())
        {
            if (timeOut_R1.checkTimeOut(true, millis))
            {
                omega += -(omega_mag);
            }
        }
        else {
            timeOut_R1.checkTimeOut(false, millis);
        }
        
        if (ps3.getButtonUp())
        {
            if (timeOut_Atas.checkTimeOut(true, millis))
            {
                vx += -(vx_mag);
            }
        }
        else {
            timeOut_Atas.checkTimeOut(false, millis);
        }

        if (ps3.getButtonDown())
        {
            if (timeOut_Bawah.checkTimeOut(true, millis))
            {
                vx += vx_mag;
            }
        }
        else {
            timeOut_Bawah.checkTimeOut(false, millis);
        }

        if (ps3.getButtonLeft())
        {
            if (timeOut_Kiri.checkTimeOut(true, millis))
            {
                vy += -(vy_mag);
            }
        }
        else {
            timeOut_Kiri.checkTimeOut(false, millis);
        }

        if (ps3.getButtonRight())
        {
            if (timeOut_Kanan.checkTimeOut(true, millis))
            {
                vy += vy_mag;
            }
        }
        else {
            timeOut_Kanan.checkTimeOut(false, millis);
        }

        

        if (ps3.getL2())
        {
            if (timeOut_L2.checkTimeOut(true, millis))
            {
                if (!toggleL2){
                    SnailMode = !SnailMode;
                    toggleL2 = true;
                }
            }
        }
        else {
            timeOut_L2.checkTimeOut(false, millis);
            toggleL2 = false;
        }

        #define SPEED_MULTIPLIER 0.75f
        #define OMEGA_MULTIPLIER 0.75f
        

        vx = SPEED_MULTIPLIER*vx;
        vy = SPEED_MULTIPLIER*vy;
        omega = OMEGA_MULTIPLIER*omega;

        if (SnailMode)
        {
            vx = vx/9.0f;
            vy = vy/9.0f;
            omega = omega/9.0f;
        }

        // OMNIWHEEL
        omniwheel.setVx(vx);
        omniwheel.setVy(vy);
        omniwheel.setOmega(omega);

        omniwheel.InverseCalc();

        // BASE KANAN
        BR_setvalue = omniwheel.getFLSpeedRPS();
        FR_setvalue = omniwheel.getBLSpeedRPS();

        // BASE KIRI
        BL_setvalue = omniwheel.getFRSpeedRPS();
        FL_setvalue = omniwheel.getBRSpeedRPS();

/*         // Set the motor speeds
        #define SPEED_MULTIPLIER 1.5f
        #define OMEGA_MULTIPLIER 1.5f

        sender.setMotor1(int(vx*1000*SPEED_MULTIPLIER));
        sender.setMotor2(int(vy*1000*SPEED_MULTIPLIER));
        sender.setInteger(int(omega*1000*OMEGA_MULTIPLIER)); */

        //------------------------------------------------
        #define ANALOG_SCALE 1000.0f


        // CAN KANAN
        can_kanan.setMotor1(BR_setvalue*ANALOG_SCALE);
        can_kanan.setMotor2(FR_setvalue*ANALOG_SCALE);

        // CAN KIRI
        can_kiri.setMotor1(BL_setvalue*ANALOG_SCALE);
        can_kiri.setMotor2(FL_setvalue*ANALOG_SCALE);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    

        // CAN STORAGE
        storage.setSwitch1(release_blue_ball);
        storage.setSwitch2(release_red_ball);
        storage.setSwitch3(separator_extended);

        // CAN GRIPPER
        gripper.setSwitch1(gripper_grip);
        gripper.setSwitch2(gripper_lift);

        can_kanan.sendCAN(can_kanan.getNoBM(), TS_SEND_CAN);
        can_kiri.sendCAN(can_kiri.getNoBM(), TS_SEND_CAN);
        storage.sendCAN(storage.getNoBM(), TS_SEND_CAN);
        gripper.sendCAN(gripper.getNoBM(), TS_SEND_CAN);
    }
    return 0;
}