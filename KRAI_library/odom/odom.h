#ifndef ODOM_H
#define ODOM_H

#include "../encoderHAL/encoderHAL.h"
#include "../encoderKRAI/encoderKRAI.h"
// #include "../../../KRAI_Library/CMPS12_KRAI/CMPS12_KRAI.h"
// #include "../../../KRAI_Library/GY80/GY80.h"
#include "Coordinate.h"

class odom{
    private:
        encoderHAL *encX_HAL;
        encoderHAL *encY_HAL;
        encoderKRAI *encX_KRAI;
        encoderKRAI *encY_KRAI;
        int xTemp, yTemp;
        bool encoderType; //true if HAL, false if KRAI
    public:
        odom(encoderHAL *encX, encoderHAL *encY);
        odom(encoderKRAI *encX, encoderKRAI *encY);
        odom(); //why not
        Coordinate position;
        void resetOdom(void);
        void updatePosition(void);
        void setPosition(float x_input, float y_input);
};

#endif