#ifndef _BOARDMANAGER_V1_PINOUT_H_
#define _BOARDMANAGER_V1_PINOUT_H_

/* ---------------- MOTOR ---------------- */
#define BMV1_PWM_MOTOR_1        PA_8
#define BMV1_FOR_MOTOR_1        PA_15
#define BMV1_REV_MOTOR_1        PB_3

#define BMV1_PWM_MOTOR_2        PA_9
#define BMV1_FOR_MOTOR_2        PB_4
#define BMV1_REV_MOTOR_2        PB_5
/* ---------------- MOTOR ---------------- */


/* -------------- ENCODER ---------------- */
#define BMV1_ENCODER_1_A        PB_12
#define BMV1_ENCODER_1_B        PB_13

#define BMV1_ENCODER_2_A        PB_14
#define BMV1_ENCODER_2_B        PB_15
/* -------------- ENCODER ---------------- */


/* ----- GUNAKAN HANYA SALAH SATU UART/I2C ----- */
#define BMV1_I2C_SCL            PB_10
#define BMV1_I2C_SDA            PB_11

#define BMV1_UART_RX            PB_11
#define BMV1_UART_TX            PB_10
/* --------------------------------------------- */


/* --------------- INTERUPT PIN --------------- */
#define BMV1_INT_1              PA_7
#define BMV1_INT_2              PA_6
#define BMV1_INT_3              PA_5            // Gabisa PWM ya mas
/* -------------------------------------------- */


#endif