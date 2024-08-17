#include "mbed.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../../KRAI_library/Motor/Motor.h"
#include "../../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../../KRAI_library/MiniPID/MiniPID.h"
#include "../../../KRAI_library/servoKRAI/servoKRAI.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"

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

// ENCODER
#define PPR 10332.0f

// PID
MiniPID pid(0.033, 0, 0.5);
uint32_t last = 0; 
uint32_t lastposition = 0;

// GRIPPER
bool sequence1 = false;
bool button_atas;
bool button_bawah;
 
int main()
{
    //TIMER

    ms_tick.attach(&onMillisecondTicker, 0.001); 

    //SERVO
    servoKRAI myServo(BMV1_INT_1);

    //MOTOR
    Motor motor(BMV1_PWM_MOTOR_2, BMV1_FOR_MOTOR_2, BMV1_REV_MOTOR_2);
    encoderKRAI enc(BMV1_ENCODER_1_A, BMV1_ENCODER_1_B, PPR, Encoding::X4_ENCODING);

    //PID
    float derajat;
    pid.setOutputLimits(-1.0, 1.0);
    float pidOutput;
    float targetPosition = 0.0; // Start at 0 degrees

    //CAN
    bool CANmessage = false;

    
    while (true)
    {
        // TERIMA DATA CAN SETIAP 100 ms
        if (gripper.readCAN(TS_READ_CAN)){
            if (millis - data_timer > 500)
            {
                led = !led;
                data_timer = millis;
            }
            can_timeout_timer = millis;
        }

        // Get gripper sequence trigger message
        if (!CANmessage)
        {
            CANmessage = gripper.getSwitch1();
        }
        

        // SEQEUENCE 1
        // gripper tutup, lengan naik, gripper buka kembali
        if (CANmessage){
            myServo.position(80);
            if (millis - lastposition >= 1000){
                if (button_atas == true ){
                    targetPosition = derajat;
                    if (millis - lastposition >= 1100){
                        myServo.position(-20);
                        sequence1 = true;
                        lastposition = millis;
                        CANmessage = false;
                    }
                }
            }
                else{
                    motor.speed(0.4);
                }
        }

        // SEQUENCE 2, berjalan jika sequence 1 selesai dilakukan (sequence1 == true)
        // Untuk gripper turun ke posisi semula
        if (sequence1){
            if(button_bawah){
                motor.speed(0);
                sequence1 = false;
            }
            else{
                motor.speed(-0.2);
            }
        }

        //  PID OUTPUT READ SETIAP 10 ms
        if (millis - last >= 10 && CANmessage == true){
            derajat = ((enc.getPulses() * 360.0f) / PPR); // dapat posisi derajat motor
            pidOutput = pid.getOutput(derajat, targetPosition); // pid out
            motor.speed(pidOutput); // set motor
            last = millis; // waktu
        } 
        printf("derajat = %f, setA = %f, setB = %f \n ", (derajat), 0.0f, 100.0f);

    }
    return 0;
}