#include "BMAktuatorKRAI.hpp"

extern CAN can;

BMAktuatorKRAI::BMAktuatorKRAI(int8_t nomer_board, uint32_t *millis) {
    this->BMData.current_bm = nomer_board;
    this->millis = millis;
}

bool BMAktuatorKRAI::sendCAN(int8_t from_bm, uint32_t time_send)
{
    if (*this->millis - this->timer_send > time_send)
    {
        this->timer_send = *this->millis;
        return (this->sendingCAN(from_bm));
    }
}

bool BMAktuatorKRAI::readCAN(uint32_t time_read)
{
    if (*this->millis - this->timer_read > time_read)
    {
        this->timer_read = *this->millis;
        return (this->readingCAN());
    }
}

void BMAktuatorKRAI::printData(uint32_t time_print)
{
    if (*this->millis - this->timer_print > time_print)
    {
        printf("Source: %d NoBM: %d isReq: %d isMode: %d Motor1: %d Motor2: %d Integer: %d SW1: %d SW2: %d SW3: %d SW4: %d SW5: %d SW6: %d\n", this->getSourceBM(), this->getNoBM(), this->getReq(), this->getMode(), this->getMotor1(), this->getMotor2(), this->getInteger(), this->getSwitch1(), this->getSwitch2(), this->getSwitch3(), this->getSwitch4(), this->getSwitch5(), this->getSwitch6());
        this->timer_print = *this->millis;
    }
}

void BMAktuatorKRAI::reset()
{
    NVIC_SystemReset();
}

bool BMAktuatorKRAI::sendingCAN(int8_t from_bm)
{   
    // JIka variabel di Class ingin mengirim atau ada perubahan data
    if (this->isAlwaysSend)
    {
        this->isWantToSend = true;
        return (this->sendmsg64(from_bm));
    }
    else if (this->isWantToSend)
    {
        // Mengecek apakah berhasil terkirim atau tidak
        this->BMData.source_bm = from_bm;
        if(this->sendmsg64(from_bm))
        {
            // JIka berhasil terkirim berikan return true dan set IsWantToSend menjadi false
            // Periksa apakah data terkiirim melebihi counter
            if (this->timeout_send_counter < this->TIMEOUT_SEND)
            {
                this->timeout_send_counter+= 1;
            }
            else
            {
                this->isWantToSend = false;
                this->timeout_send_counter = 0;
            }
            return true;                                    // Jika ada data yang ingin dikirim dan data berhasil diterima berikan return true

        } else {
            return false;                                   // Jika ada data yang ingin dikirim tetapi gagal mengirim ke target berikan return false
        }
    }
    return false;
}


bool BMAktuatorKRAI::readingCAN()
{
    if (this->readmsg64())
    {
        
        if(this->TemporaryCAN.is_req == false && this->TemporaryCAN.mode == false){
            
            // Proses Copy Data
            this->BMData.source_bm = this->TemporaryCAN.source_bm;
            this->BMData.mode = this->TemporaryCAN.mode;
            this->BMData.target_motor_1 = this->TemporaryCAN.target_motor_1;
            this->BMData.target_motor_2 = this->TemporaryCAN.target_motor_2;
            this->BMData.integer = this->TemporaryCAN.integer;
            this->BMData.switch1 = this->TemporaryCAN.switch1;
            this->BMData.switch2 = this->TemporaryCAN.switch2;
            this->BMData.switch3 = this->TemporaryCAN.switch3;
            this->BMData.switch4 = this->TemporaryCAN.switch4;
            this->BMData.switch5 = this->TemporaryCAN.switch5;
            this->BMData.switch6 = this->TemporaryCAN.switch6;
        }

        return true;
    }
    return false;
}


bool BMAktuatorKRAI::readmsg64()
{
    CANMessage msg;

    if (can.read(msg))
    {

        if ((int)msg.id == (this->BMData.current_bm))
        {
            // Ekstrak data dari pesan
            uint64_t received_data = *reinterpret_cast<uint64_t *>(msg.data);
            this->TemporaryCAN.current_bm = static_cast<int8_t>(msg.id);

            // PROSES EKSTRAKSI DATA
            this->TemporaryCAN.source_bm = static_cast<int8_t>((received_data >> 56) & 0xFF);
            this->TemporaryCAN.is_req = static_cast<bool>((received_data >> 55) & 0x01);
            this->TemporaryCAN.mode = static_cast<bool>((received_data >> 54) & 0x01);

            // Switches
            this->TemporaryCAN.switch1 = static_cast<bool>((received_data >> 53) & 0x01);
            this->TemporaryCAN.switch2 = static_cast<bool>((received_data >> 52) & 0x01);
            this->TemporaryCAN.switch3 = static_cast<bool>((received_data >> 51) & 0x01);
            this->TemporaryCAN.switch4 = static_cast<bool>((received_data >> 50) & 0x01);
            this->TemporaryCAN.switch5 = static_cast<bool>((received_data >> 49) & 0x01);
            this->TemporaryCAN.switch6 = static_cast<bool>((received_data >> 48) & 0x01);

            // Pembacaan dilakukan dalam 2'S COMPLEMENT
            this->TemporaryCAN.target_motor_1 = static_cast<int16_t>((received_data >> 32) & 0xFFFF);        // TANPA SIGN
            this->TemporaryCAN.target_motor_2 = static_cast<int16_t>((received_data >> 16) & 0xFFFF);        // TANPA SIGN
            this->TemporaryCAN.integer = static_cast<int16_t>((received_data >> 0) & 0xFFFF);                 // TANPA SIGN

            return true; // Data berhasil diekstrak
        }
    }
    return false; // Tidak ada data yang diterima atau data tidak sesuai
}


bool BMAktuatorKRAI::sendmsg64(int8_t from_bm)
{
    // Buat Kontainer berisikan 64 bit
    uint64_t message = 0;

    // Masukkan data id pengirim ke pesan
    from_bm = (from_bm &= 0xFF);                                            // Pastikan hanya terdiri dari 1 byte
    message |= (static_cast<uint64_t>(from_bm) << 56);                      // 1 byte data_id, geser ke kiri 56 bit

    // Masukkan data is request
    // True jika pengirim ingin meminta data, false jika pengirim ingin mengeset data
    message |= (static_cast<uint64_t>(this->BMData.is_req) << 55);         // KIrimkan 1 bit untuk request atau nge set parameter

    // Masukkan data mode
    message |= (static_cast<uint64_t>(this->BMData.mode) << 54);           // Kirimkan 1 bit mode

    // Masukkan data Switch
    message |= (static_cast<uint64_t>(this->BMData.switch1) << 53);
    message |= (static_cast<uint64_t>(this->BMData.switch2) << 52);
    message |= (static_cast<uint64_t>(this->BMData.switch3) << 51);
    message |= (static_cast<uint64_t>(this->BMData.switch4) << 50);
    message |= (static_cast<uint64_t>(this->BMData.switch5) << 49);
    message |= (static_cast<uint64_t>(this->BMData.switch6) << 48);

    // Masukkan data target motor 1
    this->BMData.target_motor_1 = (this->BMData.target_motor_1 &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.target_motor_1)) << 32);

    // Masukkan data target motor 2
    this->BMData.target_motor_2 = (this->BMData.target_motor_2 &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.target_motor_2)) << 16);

    // Masukkan data integer
    this->BMData.integer = (this->BMData.integer &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.integer)) << 0);

    CANMessage msg((this->BMData.current_bm), reinterpret_cast<char *>(&message), 8);
    if (can.write(msg))
    {
        return true;
    }
    return false;
}

void BMAktuatorKRAI::setMotor1( int16_t trg_motor_1 ) { 

    if(this->BMData.target_motor_1 != trg_motor_1){
        this->BMData.target_motor_1 = trg_motor_1;
        this->isWantToSend = true;
    }
}
void BMAktuatorKRAI::setMotor2( int16_t trg_motor_2 ) { 
    if(this->BMData.target_motor_2 != trg_motor_2){
        this->BMData.target_motor_2 = trg_motor_2;
        this->isWantToSend = true; 
    }
    
}
void BMAktuatorKRAI::setInteger( int16_t integer ) { 
    if(this->BMData.integer != integer){
        this->BMData.integer = integer;
        this->isWantToSend = true;  
    }
}

void BMAktuatorKRAI::setSwitch1( bool switch1 ) { 
    if(this->BMData.switch1 != switch1){
        this->BMData.switch1 = switch1;       
        this->isWantToSend = true; 
    }
}
void BMAktuatorKRAI::setSwitch2( bool switch2 ) { 
    if(this->BMData.switch2 != switch2){
        this->BMData.switch2 = switch2;       
        this->isWantToSend = true; 
    }
}
void BMAktuatorKRAI::setSwitch3( bool switch3 ) { 

    if(this->BMData.switch3 != switch3){
        this->BMData.switch3 = switch3;       
        this->isWantToSend = true;
    }
}
void BMAktuatorKRAI::setSwitch4( bool switch4 ) { 
    if(this->BMData.switch4 != switch4){
        this->BMData.switch4 = switch4;       
        this->isWantToSend = true; 
    }
}
void BMAktuatorKRAI::setSwitch5( bool switch5 ) {  
    if(this->BMData.switch5 != switch5){
        this->BMData.switch5 = switch5;       
        this->isWantToSend = true;
    }
}
void BMAktuatorKRAI::setSwitch6( bool switch6 ) { 
    if(this->BMData.switch6 != switch6){
        this->BMData.switch6 = switch6;       
        this->isWantToSend = true; 
    }
}

