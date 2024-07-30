#include "ControlHOmni.h"

ControlHOmni::ControlHOmni(masterRoda *LB_motor, masterRoda *LF_motor, masterRoda *RB_motor, masterRoda *RF_motor, masterRoda *MO_motor, odom *baseOdom, uint32_t baseSampling_us)
{    
    this->LB_motor = LB_motor;
    this->LF_motor = LF_motor;
    this->RB_motor = RB_motor;
    this->RF_motor = RF_motor;
    this->MO_motor = MO_motor;
    this->baseOdom = baseOdom;
    // this->vxPid = vxPid;
    // this->vyPid = vyPid;
    // this->wPid = wPid;
    // this->pid = pid;
    // this->pid2 = pid2;
    this->curr_dest_cout = 0;
    this->mode = 1;
    // this->line = line;
    // this->line->initialPosition(this->baseOdom->position.x, this->baseOdom->position.y, this->baseOdom->position.teta);
    // this->line->setTarget((float)(this->baseOdom->position.x + arr_x_test[0]), (float)(this->baseOdom->position.y + arr_y_test[0]));
    this->initialPos.x = this->baseOdom->position.x;
    this->initialPos.y = this->baseOdom->position.y;
    this->initialPos.teta = this->baseOdom->position.teta;
    // this->line->setError(ERROR_THRESHOLD);
    this->baseSampling_us = baseSampling_us;
    this->maxdelta_v_for = (float)this->baseSampling_us * MAX_ACCEL_FOR / 1000000.0f;
    this->maxdelta_v_right = (float)this->baseSampling_us * MAX_ACCEL_RIGHT / 1000000.0f;
    this->maxdelta_w = (float)this->baseSampling_us * MAX_ACCEL_ROT / 1000000.0f;
}

/*Getter*/
void ControlHOmni::getVars(float *v_right_cmd, float *v_for_cmd, float *w_cmd, 
                        float *v_LB_curr, float *v_LF_curr, float *v_RB_curr, float *v_RF_curr, float *v_MO_curr, 
                        float *LB_pwm, float *LF_pwm, float *RB_pwm, float *RF_pwm, float *MO_pwm, 
                        float *B_target_speed, float *F_target_speed, float *MO_target_speed, 
                        float *v_for_motor, float *v_right_motor, float *w_motor,
                        float *v_for_last, float *v_right_last, float *w_last, 
                        float *x_pos, float *y_pos, float *theta_pos, float *x_dest, float *y_dest) {
    *v_right_cmd = this->v_right_cmd;
    *v_for_cmd = this->v_for_cmd;
    *w_cmd = this->w_cmd;
    // *v_LB_curr = this->LB_motor->getSpeed();
    // *v_LF_curr = this->LF_motor->getSpeed();
    // *v_RB_curr = this->RB_motor->getSpeed();
    // *v_RF_curr = this->RF_motor->getSpeed();
    // *v_MO_curr = this->MO_motor->getSpeed();
    // *LB_pwm = this->LB_motor->getPWM();
    // *LF_pwm = this->LF_motor->getPWM();
    // *RB_pwm = this->RB_motor->getPWM();
    // *RF_pwm = this->RF_motor->getPWM();
    // *MO_pwm = this->MO_motor->getPWM();
    *B_target_speed = this->B_target_speed;
    *F_target_speed = this->F_target_speed;
    *MO_target_speed = this->MO_target_speed;
    *v_for_motor = this->v_for_motor;
    *v_right_motor = this->v_right_motor;
    *w_motor = this->w_motor;
    *v_for_last = this->v_for_last;
    *v_right_last = this->v_right_last;
    *w_last = this->w_last;
    *x_pos = this->baseOdom->position.x;
    *y_pos = this->baseOdom->position.y;
    *theta_pos = this->baseOdom->position.teta;
    // if (mode == 1) {
    //     *x_dest = this->arr_x_test[this->curr_dest_cout];
    //     *y_dest = this->arr_y_test[this->curr_dest_cout];
    // }
    // else if (mode == 2) {
    //     *x_dest = this->arr_x_test[this->curr_dest_cout];
    //     *y_dest = this->arr_y_test[this->curr_dest_cout];
    // }
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

/*Sampling*/
// void ControlHOmni::encoderMotorSamp(){
//     this->baseSpeed();
//     this->LB_motor->encMotorSamp();
//     this->LF_motor->encMotorSamp();
//     this->RB_motor->encMotorSamp();
//     this->RF_motor->encMotorSamp();
//     this->MO_motor->encMotorSamp();
// }

// void ControlHOmni::pidMotorSamp(){
//     /* menghitung pid motor base */   
//     this->LB_motor->pidMotorSamp();
//     this->LF_motor->pidMotorSamp();
//     this->RB_motor->pidMotorSamp();
//     this->RF_motor->pidMotorSamp();
//     this->MO_motor->pidMotorSamp();        
// }

// void ControlHOmni::motorSamp() {
//     this->LB_motor->motorSamp();
//     this->LF_motor->motorSamp();
//     this->RB_motor->motorSamp();
//     this->RF_motor->motorSamp();
//     this->MO_motor->motorSamp();
// }

// void ControlHOmni::samp() {
//     this->baseSpeed();
//     this->baseSamp();
//     this->LB_motor->rodaSamp();
//     this->LF_motor->rodaSamp();
//     this->RB_motor->rodaSamp();
//     this->RF_motor->rodaSamp();
//     this->MO_motor->rodaSamp();
// }

/*Procedure*/
void ControlHOmni::forceBrakeSync(bool brakingState){
    this->LB_motor->forcebrake(brakingState);
    this->LF_motor->forcebrake(brakingState);
    this->RB_motor->forcebrake(brakingState);
    this->RF_motor->forcebrake(brakingState);
    this->MO_motor->forcebrake(brakingState);

    if(brakingState){
        this->v_right_cmd = 0.0f;
        this->v_for_cmd = 0.0f;
        this->w_cmd = 0.0f;
    }
}

void ControlHOmni::MO_brake(bool brakingState){
    this->LB_motor->forcebrake(brakingState);
    this->LF_motor->forcebrake(brakingState);
    this->RB_motor->forcebrake(brakingState);
    this->RF_motor->forcebrake(brakingState);
    this->MO_motor->forcebrake(brakingState);
}

void ControlHOmni::reset(){
    this->LB_motor->reset();
    this->LF_motor->reset();
    this->RB_motor->reset();
    this->RF_motor->reset();
    this->MO_motor->reset();
}

void ControlHOmni::updatePosition(){
    // this->baseOdom->updatePosition();
    // this->line->updatePosition(this->baseOdom->position.x, this->baseOdom->position.y, this->baseOdom->position.teta, &this->setpoint, &this->feedback, &this->max);
}

void ControlHOmni::baseSpeed(){
    this->updatePosition();

    // printf("%f %f %f\t\t%d %f %f\n", this->baseOdom->position.x, this->baseOdom->position.y, this->baseOdom->position.teta, curr_dest_cout, arr_x_offline_atas_1[curr_dest_cout], arr_y_offline_atas_1[curr_dest_cout]);
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

    this->B_target_speed = this->v_right_motor + this->w_motor * LR_width;  
    this->F_target_speed = this->v_right_motor - this->w_motor * LR_width;
    this->MO_target_speed = this->v_for_motor;

    this->LB_motor->controlMotorSpeed(B_target_speed, false);
    this->LF_motor->controlMotorSpeed(F_target_speed, false);
    this->RB_motor->controlMotorSpeed(B_target_speed, false);
    this->RF_motor->controlMotorSpeed(F_target_speed, false);
    this->MO_motor->controlMotorSpeed(MO_target_speed, false);
}