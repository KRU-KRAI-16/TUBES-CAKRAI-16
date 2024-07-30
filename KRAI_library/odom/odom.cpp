/***************************************************************************
 * Title      : 
 * Name       : 
 * Version    : 
 * Author     : 
 * Date       : 
 * Description:
 *
 * 
 *
 *
 ***************************************************************************/

/******************************* library ***********************************/

#include "odom.h"

// Constants
#ifndef PI
#define PI 3.14159265359
#define PPRX 2000 // 4000 // karena x4 jadi ppr * 4 = 4000 //2053
#define PPRY 2000 // 4000 // karena x4 jadi ppr * 4 = 4000
#define p 2400
#endif

#ifndef D_RODA
#define D_RODA 5.8 // 0.06
#endif
/*************************** inisiasi class *******************************/
odom::odom(encoderHAL *encX, encoderHAL *encY)
{
    this->position.x = 0; // initiate all Value
    this->position.y = 0;
    this->position.teta = 0;
    this->encX_HAL = encX;
    this->encY_HAL = encY;
    this->encoderType = true;
}

odom::odom(encoderKRAI *encX, encoderKRAI *encY)
{
    this->position.x = 0; // initiate all Value
    this->position.y = 0;
    this->position.teta = 0;
    this->encX_KRAI = encX;
    this->encY_KRAI = encY;
    this->encoderType = false;
}

odom::odom()
{
    this->position.x = 0; // initiate all Value
    this->position.y = 0;
    this->position.teta = 0;
    this->encoderType = false;
}

/*************************** definisi fungsi ******************************/
/* update position from base */
void odom::updatePosition(void)
{
    if(this->encoderType){
        this->xTemp = this->encX_HAL->getPulses(1); /* butuh 1.5us */
        this->yTemp = this->encY_HAL->getPulses(1); /* butuh 1.5us */
    } else{
        this->xTemp = this->encX_KRAI->getPulses(); /* butuh 1.5us */
        this->yTemp = this->encY_KRAI->getPulses(); /* butuh 1.5us */
    }

    this->position.x += -((float)this->xTemp * PI * D_RODA / PPRX); // * cos(this->position.teta) - ((float)yTemp * PI * D_RODA / PPR) * -sin(this->position.teta); /* butuh 4.5 us */
    this->position.y += ((float)this->yTemp * PI * D_RODA / PPRY); // * sin(this->position.teta) - ((float)yTemp * PI * D_RODA / PPR) * cos(this->position.teta);  /* butuh 4.5 us */
    // printf("Pos  x: %f y: %f\n", this->position.x, this->position.y);
    // printf("%d %d\n", xTemp, yTemp);
}

void odom::setPosition(float x_input, float y_input)
{
    this->position.x = x_input;
    this->position.y = y_input;

    
    // printf("CompassValue set position : %f\n", this->kompass->compassValue());
}

/* to reset all the position */
void odom::resetOdom(void)
{
    this->position.x = 0; // initiate all Value
    this->position.y = 0;
}