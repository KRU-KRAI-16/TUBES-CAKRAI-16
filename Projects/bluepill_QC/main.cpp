#include "mbed.h"
#include "../../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../../KRAI_library/CanBusKRAI/BMAktuatorKRAI.hpp"
#include "../../../KRAI_library/servoKRAI/servoKRAI.h"

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
#define ID_BM_STORAGE 5

int data_timer = 0;
int can_timeout_timer = 0;
CAN can(CAN_TX, CAN_RX, 500000);

// Switch1 -> run gripper sequence (PG45)
BMAktuatorKRAI storage_BM(ID_BM_STORAGE, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
//-----------------------------------------------------------------------------------

// -------------------------------- SERVO SETUP ----------------------------------------
servoKRAI servo(BMV1_INT_1);

int main()
{
    ms_tick.attach_us(onMillisecondTicker, 1000);

    while (true)
    {
        // Blinking jika menerima data CAN
        if (storage_BM.readCAN(TS_READ_CAN))
        {
            if (millis - data_timer > 500)
            {
                led = !led;
                data_timer = millis;
            }
            can_timeout_timer = millis;
        }

        if (millis - data_timer > 2000)
            {
                led = !led;
                data_timer = millis;
            }
        
        led ? servo.position(10) : servo.position(90);
        
    }
    
    return 0;
}