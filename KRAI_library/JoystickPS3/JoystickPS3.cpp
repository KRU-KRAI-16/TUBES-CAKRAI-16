#include "JoystickPS3.h"
#include "MappingJoystick.h"

void JoystickPS3::set_send_spi(uint8_t data) {
    this->send_spi = data;
}

void JoystickPS3::set_sampling_spi(uint32_t data) {
    this->sampling_spi = data;
}

void JoystickPS3::setup()
{
    this->LX = 0; // Tadinya 127
    this->LY = 0;
    this->RX = 0;
    this->RY = 0;
    this->vx_cmd_ins = 0;
    this->vy_cmd_ins = 0;
    this->w_cmd_ins = 0;
    this->data_get = 0;
    this->is_print = 1;
    this->send_spi = 0;
    this->sampling_spi = 0;
    this->_serial.set_baud(JOYSTICK_DEFAULT_BAUD);
    this->_serial.set_format(8, BufferedSerial::None, 1);
    this->isBOHHmoving = false;
}

void JoystickPS3::olah_data()
{
    uint32_t a = this->millis;
    // Pengolahan data dari data "button"
    this->segitiga = (bool)((this->button >> 0) & 0x1);
    this->lingkaran = (bool)((this->button >> 1) & 0x1);
    this->silang = (bool)((this->button >> 2) & 0x1);
    this->kotak = (bool)((this->button >> 3) & 0x1);
    this->atas = (bool)((this->button >> 4) & 0x1);
    this->kanan = (bool)((this->button >> 5) & 0x1);
    this->bawah = (bool)((this->button >> 6) & 0x1);
    this->kiri = (bool)((this->button >> 7) & 0x1);

    // Pengolahan data dari data "RL"
    this->R1 = (bool)((this->RL >> 0) & 0x1);
    this->R3 = (bool)((this->RL >> 1) & 0x1);
    this->L1 = (bool)((this->RL >> 2) & 0x1);
    this->L3 = (bool)((this->RL >> 3) & 0x1);
    this->START = (bool)((this->RL >> 4) & 0x1);
    this->SELECT = (bool)((this->RL >> 5) & 0x1);
    this->PS = (bool)((this->RL >> 6) & 0x1);

    // R2 click dan L2 click
    if (this->R2 > 100) {
        if (this->R2sebelum) {
            this->R2_click = false;
        }
        else {
            this->R2_click = true;
        }
        this->R2sebelum = true;
    }
    else {
        this->R2sebelum = false;
        this->R2_click = false;
    }

    if (this->L2 > 100) {
        if (this->L2sebelum) {
            this->L2_click = false;
        }
        else {
            this->L2_click = true;
        }
        this->L2sebelum = true;
    }
    else {
        this->L2sebelum = false;
        this->L2_click = false;
    }

    this->segitiga_click = (bool)((this->button_click >> 0) & 0x1);
    this->lingkaran_click = (bool)((this->button_click >> 1) & 0x1);
    this->silang_click = (bool)((this->button_click >> 2) & 0x1);
    this->kotak_click = (bool)((this->button_click >> 3) & 0x1);
    this->atas_click = (bool)((this->button_click >> 4) & 0x1);
    this->kanan_click = (bool)((this->button_click >> 5) & 0x1);
    this->bawah_click = (bool)((this->button_click >> 6) & 0x1);
    this->kiri_click = (bool)((this->button_click >> 7) & 0x1);

    // Pengolahan data dari data "RL"
    this->R1_click = (bool)((this->RL_click >> 0) & 0x1);
    this->R3_click = (bool)((this->RL_click >> 1) & 0x1);
    this->L1_click = (bool)((this->RL_click >> 2) & 0x1);
    this->L3_click = (bool)((this->RL_click >> 3) & 0x1);
    this->START_click = (bool)((this->RL_click >> 4) & 0x1);
    this->SELECT_click = (bool)((this->RL_click >> 5) & 0x1);
    this->PS_click = (bool)((this->RL_click >> 6) & 0x1);
}

void JoystickPS3::idle()
{
    // Set 0
    this->button = 0;
    this->RL = 0;
    this->button_click = 0;
    this->RL_click = 0;
    this->R2_click = 0;
    this->L2_click = 0;
    this->R2 = 0;
    this->L2 = 0;
    this->RX = 0;
    this->RY = 0;
    this->LX = 0;
    this->LY = 0;
}

unsigned char JoystickPS3::getChar()
{
    unsigned char temp;
    this->_serial.read(&temp, sizeof(temp));
    return temp;
}

bool JoystickPS3::baca_data()
{
    uint32_t a = this->millis;
    bool isDone = false;
    while (this->_serial.readable() && !isDone) {
        unsigned char temp = getChar();
        switch (this->data_get) {
            case 0:
                if (temp == 0x88) {
                    this->data_get++;
                }
                break;
            case 1:
                if (temp == 0x08) {
                    this->data_get++;
                }
                else {
                    this->data_get = 0;
                    this->idle();
                    isDone = 1;
                }
                break;
            case 2:
                this->button = temp;
                this->data_get++;
                break;
            case 3:
                this->RL = temp;
                this->data_get++;
                break;
            case 4:
                this->button_click = temp;
                this->data_get++;
                break;
            case 5:
                this->RL_click = temp;
                this->data_get++;
                break;
            case 6:
                this->R2 = temp;
                this->data_get++;
                break;
            case 7:
                this->L2 = temp;
                this->data_get++;
                break;
            case 8:
                this->RX = (signed char)temp;
                this->data_get++;
                break;
            case 9:
                this->RY = (signed char)temp;
                this->data_get++;
                break;
            case 10:
                this->LX = (signed char)temp;
                this->data_get++;
                break;
            case 11:
                this->LY = (signed char)temp;
                this->data_get++;
                break;
            case 12:
                if (temp == 0x18) {
                    this->olah_data();
                    isDone = true;
                }        
                this->data_get = 0;            
                break;
            default:
                this->idle();
                break;
        }
    }
    return true;
}

bool JoystickPS3::readable()
{
    return this->_serial.readable();
    
}

// bool JoystickPS3::getMode()
// {
//     // Return true kalau otomatis
//     return this->L2;
// }

void JoystickPS3::stickState(float *vx_cmd, float *vy_cmd, float *w_cmd)
{
    /* RESET STATE */
    if (this->SELECT_click)
    {
        this->is_print = !this->is_print;
    }

    this->vx_cmd_ins = 0;
    this->vy_cmd_ins = 0;
    this->w_cmd_ins = 0;

    /* STICK ARROW STATE */
    if (this->atas) {
        if (this->bawah) {
            if (this->kanan) {
                if (this->kiri) {
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
            }
            else { // !this->kanan
                if (this->kiri) {
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
            }
        }
        else { // !this->bawah
            if (this->kanan) {
                if (this->kiri) {
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        //stick right up
                        // printf("stick right up\n");
                        this->vx_cmd_ins = VEL_TRANSLASI_HIGH;
                        this->vy_cmd_ins = VEL_TRANSLASI_HIGH;
                    }
                    else { // !this->R2
                        //stick right up
                        // printf("stick right up\n");
                        this->vx_cmd_ins = VEL_TRANSLASI_LOW;
                        this->vy_cmd_ins = VEL_TRANSLASI_LOW;
                    }
                }
            }
            else { // !this->kanan
                if (this->kiri) {
                    if (this->R2) {
                        //stick left up
                        // printf("stick left up\n");
                        this->vx_cmd_ins = -VEL_TRANSLASI_HIGH;
                        this->vy_cmd_ins = VEL_TRANSLASI_HIGH;
                    }
                    else { // !this->R1
                        //stick left up
                        // printf("stick left up\n");
                        this->vx_cmd_ins = -VEL_TRANSLASI_LOW;
                        this->vy_cmd_ins = VEL_TRANSLASI_LOW;
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        //stick up
                        // printf("stick up\n");
                        this->vx_cmd_ins = 0;
                        this->vy_cmd_ins = VEL_TRANSLASI_HIGH;
                    }
                    else { // !this->R1
                        //stick up
                        // printf("stick up\n");
                        this->vx_cmd_ins = 0;
                        this->vy_cmd_ins = VEL_TRANSLASI_LOW;
                    }
                }
            }
        }
    }
    else { // !this->atas
        if (this->bawah) {
            if (this->kanan) {
                if (this->kiri) {
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        //stick right down
                        // printf("stick right down\n");
                        this->vx_cmd_ins = VEL_TRANSLASI_HIGH;
                        this->vy_cmd_ins = -VEL_TRANSLASI_HIGH;
                    }
                    else { // !this->R2
                        //stick right down
                        // printf("stick right down\n");
                        this->vx_cmd_ins = VEL_TRANSLASI_LOW;
                        this->vy_cmd_ins = -VEL_TRANSLASI_LOW;
                    }
                }
            }
            else { // !this->kanan
                if (this->kiri) {
                    if (this->R2) {
                        //stick left down
                        // printf("stick left down\n");
                        this->vx_cmd_ins = -VEL_TRANSLASI_HIGH;
                        this->vy_cmd_ins = -VEL_TRANSLASI_HIGH;
                    }
                    else { // !this->R2
                        //stick left down
                        // printf("stick left down\n");
                        this->vx_cmd_ins = -VEL_TRANSLASI_LOW;
                        this->vy_cmd_ins = -VEL_TRANSLASI_LOW;
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        //stick down
                        // printf("stick down\n");
                        this->vx_cmd_ins = 0;
                        this->vy_cmd_ins = -VEL_TRANSLASI_HIGH;
                    }
                    else { // !this->R2
                        //stick down
                        // printf("stick down\n");
                        this->vx_cmd_ins = 0;
                        this->vy_cmd_ins = -VEL_TRANSLASI_LOW;
                    }
                }
            }
        }
        else { // !this->bawah
            if (this->kanan) {
                if (this->kiri) {
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        // to be filled
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        //stick right
                        // printf("stick right\n");
                        this->vx_cmd_ins = VEL_TRANSLASI_HIGH;
                        this->vy_cmd_ins = 0;
                    }
                    else { // !this->R2
                        //stick right
                        // printf("stick right\n");
                        this->vx_cmd_ins = VEL_TRANSLASI_LOW;
                        this->vy_cmd_ins = 0;
                    }
                }
            }
            else { // !this->kanan
                if (this->kiri) {
                    if (this->R2) {
                        //stick left
                        // printf("stick left\n");
                        this->vx_cmd_ins = -VEL_TRANSLASI_HIGH;
                        this->vy_cmd_ins = 0;
                    }
                    else { // !this->R2
                        //stick left
                        // printf("stick left\n");
                        this->vx_cmd_ins = -VEL_TRANSLASI_LOW;
                        this->vy_cmd_ins = 0;
                    }
                }
                else { // !this->kiri
                    if (this->R2) {
                        // to be filled
                    }
                    else { // !this->R2
                        if (this->L1) {
                            // to be filled
                        }
                        else { // !this->L1
                            // No input condition
                            if (this->R2)
                            {
                                // printf("no input condition, R2\n");
                                // this->vx_cmd_ins = (float)((int)this->LX - 127) * 0.5 / 127.0;
                                // this->vy_cmd_ins = (float)(127 - (int)this->LY) * 0.5 / 127.0;
                            }
                            else
                            {
                                // Tailored specifically for new joystick code
                                // this->vx_cmd_ins = (float)((int)this->LX) / 127.0;
                                // this->vy_cmd_ins = (float)(-(int)this->LY) / 127.0;
                            }
                        }
                    }
                }
            }
        }
    }

    // Stick Rotation State
    if (this->R2) {
        if (this->R1) {
            this->w_cmd_ins = -VEL_ROTASI_HIGH;
        }
        if (this->L1) {
            this->w_cmd_ins = VEL_ROTASI_HIGH;
        }
    }
    else { // !this->R2
        if (this->R1) {
            this->w_cmd_ins = -VEL_ROTASI_LOW;
        }
        if (this->L1) {
            this->w_cmd_ins =  VEL_ROTASI_LOW;
        }
    }

    *vx_cmd = this->vx_cmd_ins;
    *vy_cmd = this->vy_cmd_ins;
    *w_cmd = this->w_cmd_ins;
}

void JoystickPS3::sendSPIStickState(uint8_t* data_send_spi, uint32_t* data_sampling_spi) {
    // , bool *digL, bool *digR
    if (this->send_spi == 0 && this->millis - this->sampling_spi > 100)
    {
        // this->send_spi
        if (!this->L2) {
            if (this->segitiga) {
                this->send_spi = SEGITIGA;
            }
            else if (this->lingkaran) {
                this->send_spi = LINGKARAN;
            }
            else if (this->silang) {
                this->send_spi = SILANG;
            }
            else if (this->kotak) {
                this->send_spi = KOTAK;
            }
            else if (this->RY == -128) {
                // analog kanan ke atas
                this->send_spi = RY_UP;
            }
            else if (this->RX == 127) {
                // analog kanan ke kanan
                this->send_spi = RX_RIGHT;
            }
            else if (this->RY == 127) {
                // analog kanan ke bawah
                this->send_spi = RY_DOWN;
            }
            else if (this->RX == -128) {
                // analog kanan ke kiri
                this->send_spi = RX_LEFT;
            }
            else if (this->LY == -128) {
                // analog kiri ke atas
                this->send_spi = LY_UP;
            }
            else if (this->LX == 127) {
                // analog kiri ke kanan
                this->send_spi = LX_RIGHT;
            }
            else if (this->LY == 127) {
                // analog kiri ke bawah
                this->send_spi = LY_DOWN;
            } 
            else if (this->LX == -128){
                // analog kiri ke kiri
                this->send_spi = LX_LEFT;
            }
            else if (this->R3) {
                // analog kanan button
                this->send_spi = R3_val;
            }
            else if (this->L3){
                // analog kiri button
                this->send_spi = L3_val;
            }
            else if (this->START) {
                this->send_spi = START_val;
            }
            else if (this->SELECT) {
                this->send_spi = SELECT_val;
            }
            else {
                this->send_spi = NULL_val;
            }
        }
        // if ((this->R3)) {
        //     // || this->silang
        //     // printf("masuk R3\n");
        //     if (!this->isBOHHmoving && !this->L2) {
        //         // not mode moving, is about to turn on
        //         // printf("Turned on\n");
        //         *digL = true;
        //         *digR = true;
        //         //22222222
        //         this->isBOHHmoving = !this->isBOHHmoving;
        //         // printf("isBOHHmoving: %d\n", this->isBOHHmoving);
        //     }
        //     else if (this->isBOHHmoving && this->L2) {
        //         // is on mode moving, is about to turn off
        //         // printf("Turned off\n");
        //         *digL = false;
        //         *digR = false;
        //         //2222222
        //         this->isBOHHmoving = !this->isBOHHmoving;
        //         // printf("isBOHHmoving: %d\n", this->isBOHHmoving);
        //     }
        //     // this->isBOHHmoving = !this->isBOHHmoving;
        //     // printf("isBOHHmoving: %d\n", this->isBOHHmoving);
        // }
        // else if (this->R1 && this->isBOHHmoving) {
        //     // printf("R1 and moving\n");
        //     *digR = true;
        //     *digL = false;
        // }
        // else if (this->L1 && this->isBOHHmoving) {
        //     // printf("L1 and moving\n");
        //     *digR = false;
        //     *digL = true;
        // }
        // else {
        //     if (this->isBOHHmoving) {
        //         // printf("else moving\n");
        //         *digL = true;
        //         *digR = true;
        //     }
        //     else {
        //         // printf("else not moving\n");
        //         *digL = false;
        //         *digR = false;
        //     }
        // }
    }

    *data_send_spi = this->send_spi;
    *data_sampling_spi = this->sampling_spi;
}

bool JoystickPS3::parallelParkMode()
{
    if (this->L2){
        return (this->kiri || this -> kanan);
    }
    return false;
}

bool JoystickPS3::offsetFromPoleMode(){
    if (this->L2){
        return (this -> bawah);
    }
    return false;
}

bool JoystickPS3::getStart(){
    return this->START;
}
bool JoystickPS3::getSelect(){
	return this->SELECT;
}

bool JoystickPS3::getButtonUp(){
    return this->atas;
}
bool JoystickPS3::getButtonDown(){
    return this->bawah;
}
bool JoystickPS3::getButtonLeft(){
    return this->kiri;
}
bool JoystickPS3::getButtonRight(){
    return this->kanan;
}



bool JoystickPS3::getSegitiga(){
    return this->segitiga;
}
bool JoystickPS3::getLingkaran(){
    return this->lingkaran;
}
bool JoystickPS3::getKotak(){
    return this->kotak;
}
bool JoystickPS3::getSilang(){
    return this->silang;
}



bool JoystickPS3::getR1(){
    return this->R1;
}
bool JoystickPS3::getR2(){
    return this->R2;
}

bool JoystickPS3::getL1(){
    return this->L1;
}
bool JoystickPS3::getL2(){
    return this->L2;
}


int JoystickPS3::getRX(){
    return this->RX;
}
int JoystickPS3::getRY(){
    return this->RY;
}
int JoystickPS3::getLX(){
    return this->LX;
}
int JoystickPS3::getLY(){
    return this->LY;
}