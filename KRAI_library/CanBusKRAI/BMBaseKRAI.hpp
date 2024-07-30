#ifndef BMBASEKRAI_HPP
#define BMBASEKRAI_HPP
#include "CAN.h"
#include "mbed.h"

#define CAN_BAUD_RATE 500000
#define CAN_TIMEOUT   100           // Dalam Millis

#define CAN_RX_Bluepill             PA_11
#define CAN_TX_Bluepill             PA_12


class BMBaseKRAI
{
private:
    uint32_t timer_send;                 // Dalam Millis
    uint32_t timer_read;                 // Dalam Millis
    uint32_t timer_print;
    uint32_t *millis;

    int8_t TIMEOUT_SEND = 1;
    int8_t timeout_send_counter = 0;

    // Buat Variabel Penampung Sementara
    struct BaseBM {
        int8_t      current_bm;     // 8-bit integer
        int8_t      source_bm;      // 8-bit integer

        int16_t     FrontLeft; // 16-bit integer
        int16_t     FrontRight; // 16-bit integer
        int16_t     BackLeft;        // 16-bit integer 
        int16_t     BackRight;

        bool        isFrontLeft_negative :1;
        bool        isFrontRight_negative : 1;
        bool        isBackLeft_negative : 1;
        bool        isBackRight_negative: 1;
    };

    BaseBM TemporaryCAN;
    BaseBM BMData;

    // Mengkondisikan jika terjadi perubahan, maka kirim datanya
    bool isWantToSend;
    bool isAlwaysSend = 0;
    
    // KOMUNIKASI CAN
    /**
     * @brief Membaca data, internal dalam class
     * @return boolean apakah bisa membaca can?
    */
    bool readmsg64();

    /**
     * @brief Mengirim data, internal dalam class
     * @param target_bm : Nomor Board yang ingin dikirim
    */
    bool sendmsg64(int8_t target_bm);

    /**
     * @brief Mengirim data ke CAN BUS, digunakan secara internal dalam class
     * @param from_bm : Nomor Board yang mengirim data
    */
    bool sendingCAN(int8_t from_bm);

    /**
     * @brief Membaca data dari CAN BUS
     * @return boolean apakah bisa membaca can?
    */
    bool readingCAN(); 

public:
    /**
     * @brief Constructor
     * @param nomer_board : Nomor Board yang ingin dikirim
     * @param millis : Pointer dari millis() yang ada di main program
    */
    BMBaseKRAI(int8_t nomer_board, uint32_t *millis);

    /**
     * @brief Mengirim data ke CAN BUS, Digunakan di main program
     * @param from_bm : Nomor Board yang mengirim data
     * @param time_send : Waktu interval kirim data
     * @return boolean apakah data berhasil dikirim
    */
    bool sendCAN(int8_t from_bm, uint32_t time_send);

    /**
     * @brief Membaca data dari CAN BUS, Digunakan di main program
     * @param time_read : Waktu internal baca data
     * @return boolean apakah data berhasil terbaca
    */
    bool readCAN(uint32_t time_read);

    /**
     * @brief Melakukan print data BM
     * @param time_print : Waktu untuk interval mem-print data dalam millis
    */
    void printData(uint32_t time_print);

    /**
     * @brief Jika di set ke 1, maka BM akan selalu mengirim data sesuai dengan interval
     *        yang telah di definisikan
     * @param mode 1 : Jika ingint teru  mengirim, 0 : jika kirim hanya pada perubahan data
    */
    void IsAlwaysSend(bool mode) { this->isAlwaysSend = mode; }

    /**
     * @brief Fungsi untuk mendefinisikan, berapa banyak data yang akan dikirim melalui CAN
     *        dalam satu kali pengiriman
     * @param timeout : Banyaknya data terkirim dalam satu waktu, misal timeout = 5, maka saat
     *        pengiriman, akan dilakukan sebanyak 5x.
    */
    void setTimeoutSend( int8_t timeout ) { this->TIMEOUT_SEND = timeout; }

    /**
     * @brief Reset STM32, Semua variabel akan direset
    */
    void reset();

    // SETTER
    /**
     * @brief Set Source BM
     * @param nomer_BM : Nomer BM sekarang
    */
    void setNoBM( int8_t nomer_BM );

    /**
     * @brief Set Front Left
     * @param trg_motor_FL : Target Motor Front Left
    */
    void setFL( int16_t trg_motor_FL );

    /**
     * @brief Set Front Right
     * @param trg_motor_FR : Target Motor Front Right
    */
    void setFR( int16_t trg_motor_FR );

    /**
     * @brief Set Back Left
     * @param trg_motor_BL : Target Motor Back Left
    */
    void setBL( int16_t trg_motor_BL );
    /**
     * @brief Set Back Right
     * @param trg_motor_BR : Target Motor Back Right
    */
    void setBR( int16_t trg_motor_BR );

    // GETTER
    int getNoBM()   { return(this->BMData.current_bm); };
    
    int getFL() {return(this->BMData.FrontLeft);}
    int getFR() {return(this->BMData.FrontRight);}
    int getBL() {return(this->BMData.BackLeft);}
    int getBR() {return(this->BMData.BackRight);}


    /**
     * @brief Get Millis
     * @return Millis
    */
    uint32_t getMillis() { return *this->millis; }

};

#endif