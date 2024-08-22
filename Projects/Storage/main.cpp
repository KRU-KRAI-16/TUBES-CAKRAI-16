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


//==============================SETUP CANBUS==========================================
#define CAN_TX PA_11
#define CAN_RX PA_12
#define ID_BM_STORAGE 5

int data_timer = 0;
int can_timeout_timer = 0;
CAN can(CAN_TX, CAN_RX, 500000);

class BMStorage : public BMAktuatorKRAI
{
public:
    using BMAktuatorKRAI::BMAktuatorKRAI;
    bool getBuangBolaBiru(){ return this->getSwitch1(); }
    bool getBuangMerah(){ return this->getSwitch2(); }
    bool getToggleSeparator(){ return this->getSwitch3(); }
};

BMStorage storage_BM(ID_BM_STORAGE, &millis);

// Definisikan Lamanya Menerima data
#define TS_READ_CAN     2   // 
#define TS_SEND_CAN     5   // 
//-----------------------------------------------------------------------------------


//==============================SETUP MOTOR==========================================
const float MOTOR_SPEED_PWM = 0.4;
#define PWM_MOTOR_BIRU BMV1_PWM_MOTOR_1
#define FOR_MOTOR_BIRU BMV1_FOR_MOTOR_1
#define REV_MOTOR_BIRU BMV1_REV_MOTOR_1

#define PWM_MOTOR_MERAH BMV1_PWM_MOTOR_2
#define FOR_MOTOR_MERAH BMV1_FOR_MOTOR_2
#define REV_MOTOR_MERAH BMV1_REV_MOTOR_2

Motor motor_biru(PWM_MOTOR_BIRU, FOR_MOTOR_BIRU, REV_MOTOR_BIRU);
Motor motor_merah(PWM_MOTOR_MERAH, FOR_MOTOR_MERAH, REV_MOTOR_MERAH);
//-----------------------------------------------------------------------------------


//==============================SETUP SERVO==========================================
#define SERVO_PIN BMV1_INT_1

servoKRAI separator_servo(SERVO_PIN);
bool separator_state = false;
bool separator_button_state = false;
//-----------------------------------------------------------------------------------

int main (){

    //TIMER
    ms_tick.attach(&onMillisecondTicker, 0.001); 


    while (true){

        // Blinking jika menerima data CAN
        if (storage_BM.readCAN(TS_READ_CAN)){
            if (millis - data_timer > 500)
            {
                led = !led;
                data_timer = millis;
            }
            can_timeout_timer = millis;
        }

        // Buang bola biru
        if (storage_BM.getBuangBolaBiru()){
            motor_biru.speed(MOTOR_SPEED_PWM);
        } else {
            motor_biru.speed(0);
        }
        
        // Buang bola merah
        if (storage_BM.getBuangMerah()){
            motor_merah.speed(MOTOR_SPEED_PWM);
        } else {
            motor_merah.speed(0);
        }

        // Toggle separator
        bool separator_button_reading = storage_BM.getToggleSeparator();

        if (separator_button_reading != separator_button_state){
            separator_button_state = separator_button_reading;

            if (separator_button_state == true)
            {
                separator_state = !separator_state;
                separator_state ? separator_servo.position(0) : separator_servo.position(70);

            }
            
        }
    }
}