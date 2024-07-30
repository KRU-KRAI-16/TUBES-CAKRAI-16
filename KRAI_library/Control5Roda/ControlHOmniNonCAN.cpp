#include "ControlHOmniNonCAN.h"

ControlHOmni::ControlHOmni(odom *baseOdom, uint32_t baseSampling_us)
{    
    this->LB_speed = 0.0f;
    this->LF_speed = 0.0f;
    this->RB_speed = 0.0f;
    this->RF_speed = 0.0f;
    this->MO_speed = 0.0f;

    this->baseOdom = baseOdom;

    this->curr_dest_cout = 0;
    this->mode = 1;
    
    this->initialPos.x = this->baseOdom->position.x;
    this->initialPos.y = this->baseOdom->position.y;
    this->initialPos.teta = this->baseOdom->position.teta;

    this->baseSampling_us = baseSampling_us;
    this->maxdelta_v_for = (float)this->baseSampling_us * MAX_ACCEL_FOR / 1000.0f;
    this->maxdelta_v_right = (float)this->baseSampling_us * MAX_ACCEL_RIGHT / 1000.0f;
    this->maxdelta_w = (float)this->baseSampling_us * MAX_ACCEL_ROT / 1000.0f;

}

/*Setter*/
void ControlHOmni::set_v_right_cmd(float v_right_cmd) {
    this->v_right_cmd = v_right_cmd;
}

void ControlHOmni::set_v_for_cmd(float v_for_cmd) {
    this->v_for_cmd = v_for_cmd;
}

void ControlHOmni::set_w_cmd(float w_cmd) {
    this->w_cmd = w_cmd;
}

/*Procedure*/
void ControlHOmni::forceBrakeSync(bool brakingState){
    this->isBreak = brakingState;

    if(brakingState){
        this->v_right_cmd = 0.0f;
        this->v_for_cmd = 0.0f;
        this->w_cmd = 0.0f;
    }
}

void ControlHOmni::reset(){
    this->isReset = true;
    this->v_right_cmd = 0.0f;
    this->v_for_cmd = 0.0f;
    this->w_cmd = 0.0f;
}

void ControlHOmni::updatePosition(){
    // this->baseOdom->updatePosition();
    // this->line->updatePosition(this->baseOdom->position.x, this->baseOdom->position.y, this->baseOdom->position.teta, &this->setpoint, &this->feedback, &this->max);
}

void ControlHOmni::baseSamp() {
    // Robot jalannya lurus, ga perlu koreksi pake vc vy w PID
    this->v_for_motor = this->v_for_cmd;
    this->v_right_motor = this->v_right_cmd;
    this->w_motor = this->w_cmd;
    
    if(fabs(this->v_for_motor - this->v_for_last) > this->maxdelta_v_for){
        if (this->v_for_last > this->v_for_motor){
            this->v_for_motor = this->v_for_last - this->maxdelta_v_for;
        } else {
            this->v_for_motor = this->v_for_last + this->maxdelta_v_for;
        }
    }
    if(fabs(this->v_right_motor - this->v_right_last) > this->maxdelta_v_right){
        if (this->v_right_last > this->v_right_motor){
            this->v_right_motor = this->v_right_last - this->maxdelta_v_right;    
        }
        else{
            this->v_right_motor = this->v_right_last + this->maxdelta_v_right;
        }
    }     
    if(fabs(this->w_motor - this->w_last) > this->maxdelta_w){
        if (this->w_last > this->w_motor){
            this->w_motor = this->w_last - this->maxdelta_w;    
        }
        else{
            this->w_motor = this->w_last + this->maxdelta_w;
        }
    }   
    this->v_for_last = this->v_for_motor;
    this->v_right_last = this->v_right_motor;
    this->w_last = this->w_motor;

    this->B_target_speed = this->v_for_motor - this->w_motor;  
    this->F_target_speed = this->v_for_motor + this->w_motor;
    this->MO_target_speed = this->v_right_motor;

    this->LB_speed = B_target_speed;
    this->LF_speed = F_target_speed;
    this->RB_speed = B_target_speed;
    this->RF_speed = F_target_speed;
    this->MO_speed = MO_target_speed;

    this->LB_speed = B_target_speed;
    this->LF_speed = F_target_speed;
    this->RB_speed = B_target_speed;
    this->RF_speed = F_target_speed;
    this->MO_speed = MO_target_speed;

}