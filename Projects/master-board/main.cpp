/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
// canbus and inverse kinematics
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../../KRAI_library/InverseKinematics/Omni4Wheel.h"

// board
#include "../../../KRAI_library/MovingAverage/MovingAverage.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"

// joystick
#include "../../../KRAI_library/JoystickPS3/JoystickPS3.h"
#include "../../../KRAI_library/JoystickPS3/MappingJoystick.h"

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
#define CAN_TX PD_0
#define CAN_RX PD_1
#define ID_BM_FRONT 1
#define ID_BM_BACK 2

#define PIN_RD PA_11
#define PIN_TD PA_12

int data_timer = 0;
int can_timeout_timer = 0;
CAN can(PIN_RD, PIN_TD, 500000); //Cek lagi pin Transmit Receive di Board 1

BMAktuatorKRAI BM_front(ID_BM_FRONT, &millis);
BMAktuatorKRAI BM_back(ID_BM_BACK, &millis);

// Definisikan Lamanya Menerima dan kirim data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
 
//-------------------------UART PS3-------------------------------------------
JoystickPS3 stik (BMV1_UART_TX, BMV1_UART_RX);

//===============================Setup MOVEMENT VARIABEL=================================//


//==============================================MAIN PROGRAM======================================//
int main()
{
    //=========================INITIALIZATION====================================
    
    sender.IsAlwaysSend(1);

    ms_tick.attach_us(onMillisecondTicker,1000);
    stik.setup();

    float RawAxisL_x, RawAxisL_y;
    float RawAxisR_x;
    int VX;
    int VY;
    int Omega;


    int timedelay = 500;
    int lastmillis = millis;

    //------------------//
    int Duckspd_max = 64;
    int Duckspd_min = -64;
    int DuckOmega_max = 32;
    int DuckOmega_min = -32;

    // --------------------------------------------------------------------------

    while (true)
    {
        stik.updateMillis(millis);
        stik.baca_data();
        stik.olah_data();
     

       //==============================INPUT GETTER=========================================//
        RawAxisL_x = stik.getLX();
        RawAxisL_y = stik.getLY();
        RawAxisR_x = stik.getRX();
        
        //TO SEND
        
        VX = RawAxisL_x +=1;
        VY = RawAxisL_y +=1;
        Omega = RawAxisR_x +=1;
 
        if(stik.getR1()){
            
            
            if (VX >= Duckspd_max){
                VX = Duckspd_max;
            }
            else if (VX <= Duckspd_min){
                VX = Duckspd_min;
            }

              if (VY >= Duckspd_max){
                VY = Duckspd_max;
            }
             else if (VY <= Duckspd_min){
                VY = Duckspd_min;
            }
            
              if (Omega >= DuckOmega_max){
                Omega = DuckOmega_max;
            }
            else if (Omega<=DuckOmega_min){
                Omega = DuckOmega_min;
            }
        }
        
      

        
        
        sender.setMotor1(VX ); //BESOK BENERIN
        sender.setMotor2(VY);
        sender.setInteger(Omega);
        
        
        
        //=============================CAN BUS COMM==========================//
        sender.sendCAN(sender.getNoBM(),5);
        printf("VX: %d , VY: %d , Omega: %d \n", VX, VY, Omega);
       
        //============================USER CODE==================================
        
      

        // ----------------------------------------------------------------------
    }
    return 0;
}