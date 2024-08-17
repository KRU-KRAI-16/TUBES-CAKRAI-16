#include "mbed.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../../KRAI_library/Motor/Motor.h"
#include "../../../KRAI_library/servoKRAI/servoKRAI.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"

//============================SETUP BASIC TIMER======================================
Ticker ms_tick;
uint32_t millis = 0;
uint32_t motortime = 0;
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

// //==============================SETUP CANBUS==========================================
#define CAN_TX PA_11
#define CAN_RX PA_12
#define ID_BM_GRIPPER 2

int data_timer = 0;
int can_timeout_timer = 0;
CAN can(CAN_TX, CAN_RX, 500000);

// Switch1 -> run gripper sequence (PG45)
BMAktuatorKRAI gripper(ID_BM_GRIPPER, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
// //-----------------------------------------------------------------------------------

// float b;
// float m;

int motor1time = 0;
int motor2time = 0;
int servotime = 0;

int main (){

    //TIMER
    ms_tick.attach(&onMillisecondTicker, 0.001); 

    //SERVO
    servoKRAI myServo(BMV1_INT_1);

    //MOTOR
    Motor motor1(BMV1_PWM_MOTOR_1, BMV1_FOR_MOTOR_1, BMV1_REV_MOTOR_1);
    Motor motor2(BMV1_PWM_MOTOR_2, BMV1_FOR_MOTOR_2, BMV1_REV_MOTOR_2);

    //CAN
    // bool CANmessage = false;

    // state awal kedua flywheel
    


while (true){
    // // TERIMA DATA CAN SETIAP 100 ms
    //     if (gripper.readCAN(TS_READ_CAN)){
    //         if (millis - data_timer > 500)
    //         {
    //             led = !led;
    //             data_timer = millis;
    //         }
    //         can_timeout_timer = millis;
    //     }




        // flywheel bola merah


        // if (tombol buang merah == true) -> bola keluar
        if (millis - motor1time >= 3000){
            motor1.speed(0.2);
            if (millis - motor1time >= 6000){ // -> set flywheel mutar untuk buang berapa detik
                motor1time = millis; // tombol buang merah = false;
            }
            
        }
        // initial condition false (bola ditahan)
        else if (millis - motor1time < 3000){
            motor1.speed(-0.2);
        }

        // flywheel bola biru
        // if (tombol buang biru == true)



        if (millis - motor2time >= 3000){
            motor2.speed(0.2);
            if (millis - motor2time >= 6000){
                motor2time = millis;
            } 
        }
        else if (millis - motor2time < 3000){
            motor2.speed(-0.2);
        }
        // separator
        // if (tombol separator == true)

        if (millis - servotime >= 3000){
            myServo.position(0);
            if (millis- servotime >= 6000){
                   servotime = millis;
            }
        }
        else if (millis - servotime < 3000){
            myServo.position(90);
        }
}
}