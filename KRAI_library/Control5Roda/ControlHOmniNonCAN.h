#ifndef CONTROLHOMNI_H
#define CONTROLHOMNI_H

// ONLY FOR H BASE
// assume middle wheel is exactly in the middle

/*          CONFIGURATION
                FOR
        LB  |           |  LF
            |           |               |
                  MO                    |
                -----          right    | FB_length
                                        |
        RB  |           |  RF           |
            |           |

            -------------
               LR_width
*/

#include "../SubModuleRoda/masterRoda.h"
// #include "../CanBusKRAI/BMAktuatorKRAI.h"
#include "../odom/Coordinate.h"
#include "../odom/odom.h"
// #include "../StanleyPursuit/StanleyPursuit.h"
// #include "../../../KRAI_Library/pidLo/pidLo.h"
// #include "../../setup/Constants.h"

/*Base properties*/
#ifndef BASE_DIMENSION
#define FB_length 0.5f //in metres //assumption
#define LR_width 0.4475f //in metres
#define MAX_ACCEL_FOR 0.5f //assumption
#define MAX_ACCEL_RIGHT 0.5f //assumption
#define MAX_ACCEL_ROT (MAX_ACCEL_RIGHT / 0.25f) //assumption
#endif 

class ControlHOmni {
    protected:
        Coordinate last_pos;
        Coordinate speed_base;

        /* RODA */
        float LB_speed, LF_speed, RB_speed, RF_speed, MO_speed;
        bool isBreak;
        bool isReset;
        
        /* ODOMETRI */
        // odometriKRAI *odom;
        odom *baseOdom;

        // pidLo *vxPid;
        // pidLo *vyPid;
        // pidLo *wPid;

        Coordinate initialPos;

        /*odom2enc*/
        // StanleyPursuit *line;
        // pidLo *pid;
        // pidLo *pid2;

        int curr_dest_cout;
        int mode; // mode 1 pakai ke ball rack, mode 2 pakai ke SZ
        // Perlu 1 variabel lagi buat atas atau bawah

        float setpoint;
        float feedback;
        float max;
        float d_out;
        float v;

        /* INTERNAL VARIABLES */
        float v_right_cmd, v_for_cmd, w_cmd;
        // float v_LB_curr, v_LF_curr, v_RB_curr, v_RF_curr, v_MO_curr;
        // float LB_pwm, LF_pwm, RB_pwm, RF_pwm, MO_pwm;
        float B_target_speed, F_target_speed, MO_target_speed;
        float v_right_last, v_for_last, w_last;
        float v_for_motor, v_right_motor, w_motor;
        uint32_t baseSampling_us;
        float maxdelta_v_for, maxdelta_v_right, maxdelta_w;

    public:
        ControlHOmni(odom *baseOdom, uint32_t baseSampling_us);

        // Setter
        void set_v_right_cmd(float v_right_cmd); //right target speed in m/s
        void set_v_for_cmd(float v_for_cmd); //forward target speed in m/s
        void set_w_cmd(float w_cmd); //rotation target speed in rad/s

        // sampling
        // void samp();
        // void encoderMotorSamp(); //sampling reference 5000
        // void pidMotorSamp(); //sampling reference 4173
        // void motorSamp(); //samp reference 5173

        //prosedur
        void forceBrakeSync(bool brakingState);
        void reset();

        /* 
        * prosedur untuk melakukan sampling enc motor
        *
        *abstract method, akan dioverride tergantung child class (ada Constant spesific)*/
        virtual void updatePosition(); //sampling for odom
        virtual void baseSamp(); //sampling for each motor's target speed based on max acceleration //samp reference 20000

        /**
         * Getter
        */
        float get_v_right_cmd() { return this->v_right_cmd; }
        float get_v_for_cmd() { return this->v_for_cmd; }
        float get_w_cmd() { return this->w_cmd; }

        float get_LB_speed() { return this->LB_speed; }
        float get_LF_speed() { return this->LF_speed; }
        float get_RB_speed() { return this->RB_speed; }
        float get_RF_speed() { return this->RF_speed; }
        float get_MO_speed() { return this->MO_speed; }

};

#endif