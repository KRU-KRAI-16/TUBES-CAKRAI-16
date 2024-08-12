/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../KRAI_library/Motor/Motor.h"
#include "../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_library/JoystickPS3/JoystickPS3.h"
#include "../../KRAI_library/JoystickPS3/MappingJoystick.h"

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

 






// Definisikan Lamanya Menerima dan kirim data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
 
//-------------------------UART PS3-------------------------------------------
JoystickPS3 stik (BMV1_UART_TX,BMV1_UART_RX);

//===============================Setup MOVEMENT VARIABEL=================================//


int FL, FR; //test 001 depan, tambahin BL BR



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
    bool ToGrip =  false;

    // --------------------------------------------------------------------------

    while (true)
    {
        stik.updateMillis(millis);
        stik.baca_data();
        stik.olah_data();
        ToGrip = false;
     

       //==============================INPUT GETTER=========================================//
        RawAxisL_x = stik.getLX();
        RawAxisL_y = stik.getLY();
        RawAxisR_x = stik.getRX();
        ToGrip = stik.getKotak();

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
        
        //=====================================Code gripper=======================//
        if (stik.getKotak()){
            ToGrip = true;
        }

        
        
        sender.setMotor1(VX ); //BESOK BENERIN
        sender.setMotor2(VY);
        sender.setInteger(Omega);
        
        gripper.setSwitch1(ToGrip);
        
        
        //=============================CAN BUS COMM==========================//
    
       sender.sendCAN(sender.getNoBM(),5);
       gripper.sendCAN(gripper.getNoBM(),5);
        //============================USER CODE==================================
        
      

        // ----------------------------------------------------------------------
    }
    return 0;
}