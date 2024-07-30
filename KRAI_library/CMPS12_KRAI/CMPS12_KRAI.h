#ifndef _CMPS12_KRAI_H_
#define _CMPS12_KRAI_H_

#include "mbed.h"

#define CMPS_DEFAULT_I2C_ADDRESS 0xC0
//set register kompas
#define SOFTWARE_REVISION_REG    0x0
#define COMPASS_BEARING_WORD_REG 0x2
#define COMPASS_BEARING16_WORD_REG 0x1A
#define COMPASS_PITCH_WORD_REG 0x1C
#define COMPASS_ROLL_WORD_REG 0x5

//note: sumbu X ke atas tulisan CMPS12, sumbu Y ke kiri, dan sumbu Z menembus
#define ACCEL_X_WORD_REG 0x0C
#define ACCEL_Y_WORD_REG 0x0E
#define ACCEL_Z_WORD_REG 0x10

#define MAGNET_X_WORD_REG 0x06
#define MAGNET_Y_WORD_REG 0x08
#define MAGNET_Z_WORD_REG 0x0A

#define GYRO_X_WORD_REG 0x12
#define GYRO_Y_WORD_REG 0x14
#define GYRO_Z_WORD_REG 0x16

#define TEMP_WORD_REG 0x18

class CMPS12_KRAI {
    private:
        I2C* i2c;
        int i2cAddress;
        float _offset_compass_value;
        float _theta_origin;
        float _theta_offset;

    public:
        /** Create CMPS12 inteface
         * @param sda mbed pin to use for I2C SDA
         * @param scl mbed pin to use for I2C SCL
         * @param address I2C address of this device. (default 0xC0)
         */
        CMPS12_KRAI(PinName sda, PinName scl, int address);

        //run this before every instruction
        //return calibration status (from 0 (uncalibrated) to 8 (full calibrated))
        //return -1 if disconnected and will automaticaly try to reconnect (if autoReconnect is true)
        int status(bool autoReconnect);

        /** Membaca nilai dari kompas (sudut terhadap sumbu z0 (vertikal ke bwh))
         * 
         * Range nilai 0 - 3599
         */
        uint16_t getAngle(void);
        uint16_t getAngle16(void); //range nilai 0-5759 (bagi 16 utk derajat)
        
        /** Membaca nilai sudut pitch dari kompas (sudut terhadap x0 (horizontal ke atas tulisan cmps12))
         * 
         * Range nilai (+/-180)
         */
        int8_t getPitch(void);
        
        /** Membaca nilai sudut roll dari kompas (sudut terhadap y0 (horizontal ke kiri tulisan cmps12))
         * 
         * Range nilai (+/-90)
         */
        int8_t getRoll(void);

        //return calibration status in char 
        //note: CMPS12 is always callibrating automatically while running
        //return (0bAABBCCDD where AA is system status, BB is gyro status, CC is acc status, DD is magn status)
        char getCalibrate(void);

        //reset the I2C
        //note: CMPS12 can't be reseted via software. You need to turn it off an on
        void reset(void);

        //Use the two functions bellow if the auto calibration result is shit. (do a recalibration from 0. Be sure to turn it around)
        // Kalibrasi kompas
        void calibrate(void);
        // Stop calibrate
        void stopCalibrate(void);

        //These two are still under development
        // void saveCallibration(void);
        // void deleteCallibration(void);
        
        //getting raw sensor data
        //acceleration in cm/s^2
        int16_t getAccelX(void);
        int16_t getAccelY(void);
        int16_t getAccelZ(void);

        int16_t getMagnetX(void);
        int16_t getMagnetY(void);
        int16_t getMagnetZ(void);

        int16_t getGyroX(void);
        int16_t getGyroY(void);
        int16_t getGyroZ(void);

        int16_t getTemp(void); //in degrees centrigrade
        

        // Reset offset value from compass, set the initial condition from compass to be reference theta
        void compassResetOffsetValue();

        // Update value from compass using theta reference
        void compassUpdateValue();

        // Membaca revisi dari kompas
        char readSoftwareRevision(void);

        // Get the compass value, theta will be in degree from -180 to 180
        float compassValue();

        void changeThetaValue(float theta_robot);
};

#endif