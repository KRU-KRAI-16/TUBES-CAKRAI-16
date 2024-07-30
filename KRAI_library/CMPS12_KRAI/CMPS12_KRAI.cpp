#include "CMPS12_KRAI.h"

const float  radian_to_degree = 57.295779;

CMPS12_KRAI::CMPS12_KRAI(PinName sda, PinName scl, int address) {
    i2c = new I2C(sda, scl);
    //CMPS11 maksimum 100kHz CMPS12 maksimum 400kHz
    i2c->frequency(100000);
    this->i2cAddress = address;
    this->_offset_compass_value = 0;
    this->_theta_origin = 0;
    this->_theta_offset = 0;
}

int CMPS12_KRAI::status(bool autoReconnect){
    uint8_t statusIn = (uint8_t)this->getCalibrate();
    if(statusIn == 0b11000000){
        if(autoReconnect){
            this->reset();
        }
        return -1;
    }
    int percent = 0;
    for(int i = 0; i<8; i++){
        if((statusIn>>i) % 2 == 1){
            percent += 1;
        }
    }
    return percent;
}

char CMPS12_KRAI::readSoftwareRevision(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char registerContents = 0;

    //First, send the number of register we wish to read,
    //in this case, command register, number 0.
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Now, read one byte, which will be the contents of the command register.
    i2c->read(i2cAddress, &registerContents, 1);
    
    return registerContents; 
}

uint16_t CMPS12_KRAI::getAngle(void){
    char registerNumber = COMPASS_BEARING_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1); //1
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //Register 0 adalah 8 bit, harus di shift
    //Register 1 adalah 16 bit, bisa langsung dibaca
    uint16_t angle = ((int16_t)registerContents[0] << 8) | ((uint8_t)registerContents[1]);
    
    //kirim data berupa 1/10 derajat sudut 0-3600
    return angle;   
}

uint16_t CMPS12_KRAI::getAngle16(void){
    char registerNumber = COMPASS_BEARING16_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1); //1
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //Register 0 adalah 8 bit, harus di shift
    //Register 1 adalah 16 bit, bisa langsung dibaca
    uint16_t angle = ((int16_t)registerContents[0] << 8) | ((uint8_t)registerContents[1]);
    
    //kirim data berupa 1/16 derajat sudut 1-5759
    return angle;   
}

int8_t CMPS12_KRAI::getPitch(void){
    char registerNumber = COMPASS_PITCH_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1); //1
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //Register 0 adalah 8 bit, harus di shift
    //Register 1 adalah 16 bit, bisa langsung dibaca
    int16_t pitch = ((int16_t)registerContents[0] << 8) | ((uint8_t)registerContents[1]);
    
    return pitch;   
}

int8_t CMPS12_KRAI::getRoll(void){
    char registerNumber = COMPASS_ROLL_WORD_REG;
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 1);
    
    //simpan data ke variable roll
    int8_t roll = ((int8_t)registerContents[0] );
    
    return roll;   
}

void CMPS12_KRAI::calibrate(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char calibrate_data1 = 0xF0;
    char calibrate_data2 = 0xF5;
    char calibrate_data3 = 0xF7;
    //kirim data 1
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data1, 1);
    //kirim data 2 delay 25ms
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data2, 1);
    //kirim data 3 delay 25ms
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data3, 1);
}

char CMPS12_KRAI::getCalibrate(void){
    char calibrate_data1 = 0x1E;
    if(i2c->write(i2cAddress, &calibrate_data1, 1) != 0){
        return (char)0b11000000; //should normally be impossible
    }
    if(i2c->read(i2cAddress, &calibrate_data1, 1) != 0){
        return (char)0b11000000; //fail reading
    }
    return calibrate_data1;
}

void CMPS12_KRAI::reset(void){
    i2c->frequency(100000);
}

void CMPS12_KRAI::stopCalibrate(void){
    char registerNumber   = SOFTWARE_REVISION_REG;
    char calibrate_data1 = 0xF8;
    //kirim data 1
    i2c->write(i2cAddress, &registerNumber, 1);
    i2c->write(i2cAddress, &calibrate_data1, 1);
}

int16_t CMPS12_KRAI::getAccelX(void){
    char registerNumber = ACCEL_X_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable accelx
    int16_t accelX = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return accelX;   
}

int16_t CMPS12_KRAI::getAccelY(void){
    char registerNumber = ACCEL_Y_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable accely
    int16_t accelY = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return accelY;   
}

int16_t CMPS12_KRAI::getAccelZ(void){
    char registerNumber = ACCEL_Z_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable accel z
    int16_t accelZ = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return accelZ;   
}

int16_t CMPS12_KRAI::getMagnetX(void){
    char registerNumber = MAGNET_X_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable magnetx
    int16_t magnetX = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return magnetX;   
}

int16_t CMPS12_KRAI::getMagnetY(void){
    char registerNumber = MAGNET_Y_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable magnetY
    int16_t magnetY = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return magnetY;   
}

int16_t CMPS12_KRAI::getMagnetZ(void){
    char registerNumber = MAGNET_Z_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable magnetZ
    int16_t magnetZ = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return magnetZ;   
}

int16_t CMPS12_KRAI::getGyroX(void){
    char registerNumber = GYRO_X_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable gyroX
    int16_t gyroX = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return gyroX;   
}

int16_t CMPS12_KRAI::getGyroY(void){
    char registerNumber = GYRO_Y_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable gyroY
    int16_t gyroY = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return gyroY;   
}

int16_t CMPS12_KRAI::getGyroZ(void){
    char registerNumber = GYRO_Z_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable gyroZ
    int16_t gyroZ = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return gyroZ;   
}

int16_t CMPS12_KRAI::getTemp(void){
    char registerNumber = TEMP_WORD_REG; //accel X
    char registerContents[2] = {0x00, 0x00};
    
    //Mengirim register untuk address pertama dari i2c
    i2c->write(i2cAddress, &registerNumber, 1);
    
    //Mengambil data dari 2 address I2c
    i2c->read(i2cAddress, registerContents, 2);
    
    //simpan data ke variable temperature
    int16_t temperature = ((int16_t)registerContents[0]<<8 | (uint8_t)registerContents[1] );
    
    return temperature;   
}

void CMPS12_KRAI::compassResetOffsetValue(){
    this->_offset_compass_value = (float)CMPS12_KRAI::getAngle()/10;
    // printf("Offset value: %f\n", this->_offset_compass_value);
}

void CMPS12_KRAI::compassUpdateValue(){
    this->_theta_origin = (float)CMPS12_KRAI::getAngle()/10;
    this->_theta_offset = this->_theta_origin - this->_offset_compass_value;
    // printf("Theta origin: %f\n", this->_theta_origin);
    // printf("Theta offset: %f\n", this->_theta_offset);
}

float CMPS12_KRAI::compassValue(){
    float theta_transformed;

    if(this->_theta_offset > 180.0f && this->_theta_offset <= 360.0f)
        theta_transformed = (this->_theta_origin - 360.0f - this->_offset_compass_value)/-radian_to_degree;
    else if(this->_theta_offset < -180.0f && this->_theta_offset >= -360.0f)
        theta_transformed = (this->_theta_origin  + 360.0f - this->_offset_compass_value)/-radian_to_degree;
    else
        theta_transformed = (this->_theta_origin - this->_offset_compass_value)/-radian_to_degree;
    
    // printf("Theta transformed: %f\n", theta_transformed);

    return theta_transformed; 
}

void CMPS12_KRAI::changeThetaValue(float theta_robot){
   
    if (theta_robot >= -180 && theta_robot < 0){
        theta_robot = 360 + theta_robot;
    }

    _theta_offset = _theta_origin - theta_robot;

    if (_theta_offset < 0){
        _theta_offset += 360;
    }
}