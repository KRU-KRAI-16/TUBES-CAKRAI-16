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
#include "../../KRAI_library/Motor/Motor.h"
#include "../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
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
JoystickPS3 ps3 (BMV1_UART_TX,BMV1_UART_RX);
class TimeOutPS3
{
private:
    uint32_t prevTime = 0;
    uint32_t tempIntegral = 0;
    uint32_t durasiIgnore = 300; // ms
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
TimeOutPS3 timeOut_Kanan(200);
TimeOutPS3 timeOut_Kiri(200);
TimeOutPS3 timeOut_Atas(200);
TimeOutPS3 timeOut_Bawah(200);
TimeOutPS3 timeOut_CCW(200);
TimeOutPS3 timeOut_CW(200);



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
    float VX;
    float VY;
    int Omega;


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

    // --------------------------------------------------------------------------

    int PS3_OFFSET = 20;
    int SPEED_MS_XY = 2;
    int SPEED_LIMIT_SLOW = 1;
    int SPEED_LIMIT_FAST = 2;
    

    while (true)
    {
        ps3.updateMillis(millis);
        ps3.baca_data();
        ps3.olah_data();
        ToGrip = false;
        Isduck = false;
     

       //==============================INPUT GETTER=========================================/
        VX = 0;
        VY = 0;
        Omega = 0;
        //==========================================INPUT NO  JITTER==========================//
        isDebugPS3 = false;
        if (isDebugPS3)
        {
            printf("RX : %d RY : %d LX : %d LY : %d", ps3.getRX(), ps3.getRY(), ps3.getLX(), ps3.getLY());
            printf("R1 : %d R2 : %d L1 : %d L2 : %d ", ps3.getR1(), ps3.getR2(), ps3.getL1(), ps3.getL2());
            printf("Kotak : %d Silang : %d Lingkaran : %d Segitiga : %d", ps3.getKotak(), ps3.getSilang(), ps3.getLingkaran(), ps3.getSegitiga());
            printf("Up : %d Left : %d Right : %d Down : %d ", ps3.getButtonUp(), ps3.getButtonLeft(), ps3.getButtonRight(), ps3.getButtonDown());
            printf("Start : %d Select : %d\n", ps3.getStart(), ps3.getSelect());
        }

         // Untuk Kanan
        if(ps3.getLX() > PS3_OFFSET)
        {
            if(timeOut_Kanan.checkTimeOut(true, millis)){
                    // Mapping LX dari 0 sampai 128 menjadi dalam m/s dari 0 sampai 1
            float deltaVRight = (map(PS3_OFFSET, 128.0, 0, -(SPEED_MS_XY  * 0.5), ps3.getLX()));
            // if (deltaVRight>DeltaMAX){
            //     deltaVRight = DeltaMAX;
            // }
                VX += deltaVRight;

            // timeOut_L2.updateTime(millis);

            printf("Delta = %f \n ", deltaVRight);
            }
            
        
        }
        else {
            timeOut_Kanan.checkTimeOut(false, millis);
        }

        // Untuk Kiri
        if(ps3.getLX() < -PS3_OFFSET)
        {
            if(timeOut_Kiri.checkTimeOut(true, millis)){
            float deltaVLeft = (map(-PS3_OFFSET, -128.0, 0, (SPEED_MS_XY * 0.5), ps3.getLX()));
            // if(deltaVLeft>DeltaMAX){
            //     deltaVLeft=DeltaMAX;
            // }
            VX += deltaVLeft; 
            
            }
            // Mapping LX dari 0 sampai -128 menjadi dalam m/s dari 0 sampai 1
          
           
        }
        else {
            timeOut_Kiri.checkTimeOut(false, millis);
          
        }

      
        printf("VX = %f \n ", VX);

        //Untuk ATAS
        if(ps3.getLY() > PS3_OFFSET)
        {
            if(timeOut_Atas.checkTimeOut(true, millis)){
            // Mapping LX dari 0 sampai 128 menjadi dalam m/s dari 0 sampai 1
            // float deltaVRight = (map(PS3_OFFSET, 128.0, 0, -(SPEED_MS_XY  * 0.5), ps3.getLX()));
            float deltaVUP = (map(PS3_OFFSET, 128.0, 0, -(SPEED_MS_XY  * 0.5), ps3.getLY()));

            
            VY += deltaVUP;

            

            
            }
            
        
        }
        else {
            timeOut_Atas.checkTimeOut(false, millis);
        }


        //UNTUK BAWAH
        if(ps3.getLY() < -PS3_OFFSET)
        {
            if(timeOut_Bawah.checkTimeOut(true, millis)){
            float deltaVDown = (map(-PS3_OFFSET, -128.0, 0, (SPEED_MS_XY * 0.5), ps3.getLY()));
            // if(deltaVLeft>DeltaMAX){
            //     deltaVLeft=DeltaMAX;
            // }
            VY += deltaVDown; 
            
            }
            // Mapping LX dari 0 sampai -128 menjadi dalam m/s dari 0 sampai 1
          
           
        }
        else {
            timeOut_Bawah.checkTimeOut(false, millis);
          
        }


    
        
        //===============untuk ROTASI-================//

        //kayanya CCW DAH
        if(ps3.getRX() > PS3_OFFSET)
        {
            if(timeOut_CCW.checkTimeOut(true, millis)){
            // Mapping LX dari 0 sampai 128 menjadi dalam m/s dari 0 sampai 1
            // float deltaVRight = (map(PS3_OFFSET, 128.0, 0, -(SPEED_MS_XY  * 0.5), ps3.getLX()));
            float deltaCCW = (map(PS3_OFFSET, 128.0, 0, -(SPEED_MS_XY  * 0.5), ps3.getRX()));

            
            Omega += deltaCCW;

            }
            
        
        }
        else {
            timeOut_CCW.checkTimeOut(false, millis);
        }

        //KAYANYA CW
        if(ps3.getRX() < -PS3_OFFSET)
        {
            if(timeOut_CW.checkTimeOut(true, millis)){
            float deltaCW = (map(-PS3_OFFSET, -128.0, 0, (SPEED_MS_XY * 0.5), ps3.getRX()));
            // if(deltaVLeft>DeltaMAX){
            //     deltaVLeft=DeltaMAX;
            // }
            Omega += deltaCW; 
            
            }
            // Mapping LX dari 0 sampai -128 menjadi dalam m/s dari 0 sampai 1
          
           
        }
        else {
            timeOut_CW.checkTimeOut(false, millis);
          
        }
 



        // if(Isduck){
            
            
        //     if (VX >= Duckspd_max){
        //         VX = Duckspd_max;
        //     }
        //     else if (VX <= Duckspd_min){
        //         VX = Duckspd_min;
        //     }

        //     if (VY >= Duckspd_max){
        //         VY = Duckspd_max;
        //     }
        //     else if (VY <= Duckspd_min){
        //         VY = Duckspd_min;
        //     }
            
        //     if (Omega >= DuckOmega_max){
        //         Omega = DuckOmega_max;
        //     }
        //     else if (Omega<=DuckOmega_min){
        //         Omega = DuckOmega_min;
        //     }


        // }
        
        // printf("vx: %f, vy: %f, omega: %f, Kotak: %d\n", VX, VY, Omega, stik.getKotak());

        //=====================================Code gripper=======================//
        //if (stik.getKotak()){
          //  ToGrip = true;
        //}

        
        /////////////////////////////////////////////////
        sender.setMotor1(VX ); //BESOK BENERIN
        sender.setMotor2(VY);
        sender.setInteger(Omega);
        
        //gripper.setSwitch1(ToGrip);
        
        
        //=============================CAN BUS COMM==========================//
    
       sender.sendCAN(sender.getNoBM(),5);
       gripper.sendCAN(gripper.getNoBM(),5);
        //============================USER CODE==================================
         
      

        // ----------------------------------------------------------------------
    }
    return 0;
}