#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "mbed.h"

// Documentation : http://www.yahboom.net/study/SDM15

class LidarYD15{
public:

    /**
     * @brief Construct a new Lidar YD15 object
     * @param _rx_pin RX pin
     * @param _tx_pin TX pin
     * @param _baudrate optional, default 460800
    */
    Lidar YD15(PinName _rx_pin,PinName _tx_pin, int _baudrate: 460800);
    int getDistance();
    int getStrength();
    int getInterference();

    // update measurement harus selalu dipanggil sebelum getDistance, getStrength, getInterference
    void updateMeasurement();

private:

    BufferedSerial _serial;
    int _baudrate;
    int _distance;
    int _strength;
    int _interference;
    int _frequency;

    char start_byte[2] = {0xAA, 0x55};

    void startRanging();
    void stopRanging();
    void setOutputFrequency(int _frequency);
    void setBaudrate(int _baudrate);


};

#endif