#include "BMBaseKRAI.hpp"

extern CAN can;

BMBaseKRAI::BMBaseKRAI(int8_t nomer_board, uint32_t *millis) {
    this->BMData.current_bm = nomer_board;
    this->millis = millis;

    // Reset all integer data to 0
    this->BMData.FrontLeft = 0;
    this->BMData.FrontRight = 0;
    this->BMData.BackLeft = 0;
    this->BMData.BackRight = 0;
}

bool BMBaseKRAI::sendCAN(int8_t from_bm, uint32_t time_send)
{
    if (*this->millis - this->timer_send > time_send)
    {
        this->timer_send = *this->millis;
        return (this->sendingCAN(from_bm));
    }
}

bool BMBaseKRAI::readCAN(uint32_t time_read)
{
    if (*this->millis - this->timer_read > time_read)
    {
        this->timer_read = *this->millis;
        return (this->readingCAN());
    }
}

void BMBaseKRAI::printData(uint32_t time_print)
{
    if (*this->millis - this->timer_print > time_print)
    {
        printf("BM %d : FL %d, FR %d, BL %d, BR %d\n", this->BMData.current_bm, this->BMData.FrontLeft, this->BMData.FrontRight, this->BMData.BackLeft, this->BMData.BackRight);
        this->timer_print = *this->millis;
    }
}

void BMBaseKRAI::reset()
{
    NVIC_SystemReset();
}

bool BMBaseKRAI::sendingCAN(int8_t from_bm)
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


bool BMBaseKRAI::readingCAN()
{
    if (this->readmsg64()) {
        return true; }
    return false;
}


bool BMBaseKRAI::readmsg64()
{
    CANMessage msg;

    if (can.read(msg))
    {

        if ((int)msg.id == (this->BMData.current_bm))
        {
            // Ekstrak data dari pesan
            uint64_t received_data = *reinterpret_cast<uint64_t *>(msg.data);

            // Pembacaan dilakukan dalam 2'S COMPLEMENT
            this->BMData.BackLeft = static_cast<int16_t>((received_data >> 48) & 0xFFFF);
            this->BMData.BackRight = static_cast<int16_t>((received_data >> 32) & 0xFFFF);
            this->BMData.FrontLeft = static_cast<int16_t>((received_data >> 16) & 0xFFFF);
            this->BMData.FrontRight = static_cast<int16_t>((received_data >> 0) & 0xFFFF);

            return true; // Data berhasil diekstrak
        }
    }
    return false; // Tidak ada data yang diterima atau data tidak sesuai
}


bool BMBaseKRAI::sendmsg64(int8_t from_bm)
{
    // Buat Kontainer berisikan 64 bit
    uint64_t message = 0;

    // Masukkan data front left
    this->BMData.FrontLeft = (this->BMData.FrontLeft &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.FrontLeft)) << 48);

    // Masukkan data front right
    this->BMData.FrontRight = (this->BMData.FrontRight &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.FrontRight)) << 32);

    // Masukkan data back left
    this->BMData.BackLeft = (this->BMData.BackLeft &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.BackLeft)) << 16);

    // Masukkan data back right
    this->BMData.BackRight = (this->BMData.BackRight &= 0xFFFF);        // Pastikan hanya 2 byte
    message |= (static_cast<uint64_t>(static_cast<uint16_t>(this->BMData.BackRight)) << 0);

    CANMessage msg((this->BMData.current_bm), reinterpret_cast<char *>(&message), 8);
    if (can.write(msg))
    {
        return true;
    }
    return false;
}

void BMBaseKRAI::setFL(int16_t FrontLeft){
    if(this->BMData.FrontLeft != FrontLeft){
        this->BMData.FrontLeft = FrontLeft;
        this->isWantToSend = true;
    }
}

void BMBaseKRAI::setFR(int16_t FrontRight){
    if(this->BMData.FrontRight != FrontRight){
        this->BMData.FrontRight = FrontRight;
        this->isWantToSend = true;
    }
}

void BMBaseKRAI::setBL(int16_t BackLeft){
    if(this->BMData.BackLeft != BackLeft){
        this->BMData.BackLeft = BackLeft;
        this->isWantToSend = true;
    }
}

void BMBaseKRAI::setBR(int16_t BackRight){
    if(this->BMData.BackRight != BackRight){
        this->BMData.BackRight = BackRight;
        this->isWantToSend = true;
    }
}