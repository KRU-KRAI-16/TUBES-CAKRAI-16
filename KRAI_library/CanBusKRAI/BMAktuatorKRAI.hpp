#ifndef BMAKUATORKRAI_HPP
#define BMAKUATORKRAI_HPP
#include "CAN.h"
#include "mbed.h"

#define CAN_BAUD_RATE 1000000
#define CAN_TIMEOUT   100           // Dalam Millis

#define CAN_RX_Bluepill             PA_11
#define CAN_TX_Bluepill             PA_12


class BMAktuatorKRAI
{
private:
    uint32_t timer_send;                 // Dalam Millis
    uint32_t timer_read;                 // Dalam Millis
    uint32_t timer_print;
    uint32_t *millis;

    int8_t TIMEOUT_SEND = 1;
    int8_t timeout_send_counter = 0;

    // Buat Variabel Penampung Sementara
    struct AktuatorBM {
        int8_t      current_bm;     // 8-bit integer
        int8_t      source_bm;      // 8-bit integer
        bool        is_req  : 1;    // 1-bit boolean    (false jika ingin meng-set nilai, true jika hanya ingin cek koneksi)
        bool        mode    : 1;    // 1-bit boolean    default false(0)
        int16_t     target_motor_1; // 16-bit integer
        int16_t     target_motor_2; // 16-bit integer
        int16_t     integer;        // 16-bit integer 
        bool        switch1 : 1;    // 1-bit boolean
        bool        switch2 : 1;    // 1-bit boolean
        bool        switch3 : 1;    // 1-bit boolean
        bool        switch4 : 1;    // 1-bit boolean
        bool        switch5 : 1;    // 1-bit boolean
        bool        switch6 : 1;    // 1-bit boolean

        bool        isMotor1_negative :1;
        bool        isMotor2_negative :1;
        bool        isInteger_negative :1;
    };

    AktuatorBM TemporaryCAN;
    AktuatorBM BMData;

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
    BMAktuatorKRAI(int8_t nomer_board, uint32_t *millis);

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
    void setNoBM( int8_t nomer_BM ) { this->BMData.current_bm = nomer_BM;   this->isWantToSend = true; }
    /**
     * @brief Set Request
     * @param is_request : True jika hanya ingin cek koneksi, False jika ingin mengirim data
    */
    void setReq( bool is_request ) { this->BMData.is_req = is_request;      this->isWantToSend = true; }
    /**
     * @brief Set Mode
     * @param mode : mode default false
    */
    void setMode( bool mode ) { this->BMData.mode = mode;                   this->isWantToSend = true; }

    /**
     * @brief Set Motor 1
     * @param trg_motor_1 : Target Motor 1
    */
    void setMotor1( int16_t trg_motor_1 );
    /**
     * @brief Set Motor 2
     * @param trg_motor_2 : Target Motor 2
    */
    void setMotor2( int16_t trg_motor_2 );
    /**
     * @brief Set Integer
     * @param integer : Integer
    */
    void setInteger( int16_t integer );

    void setSwitch1( bool switch1 );
    void setSwitch2( bool switch2 );
    void setSwitch3( bool switch3 );
    void setSwitch4( bool switch4 );
    void setSwitch5( bool switch5 );
    void setSwitch6( bool switch6 );

    // GETTER
    int getSourceBM()   { return(this->BMData.source_bm); };
    int getNoBM()   { return(this->BMData.current_bm); };
    
    bool getReq()   { return(this->BMData.is_req); };
    bool getMode()  { return(this->BMData.mode); };
    
    int getMotor1() { return(this->BMData.target_motor_1); };
    int getMotor2() { return(this->BMData.target_motor_2); };
    int getInteger() { return(this->BMData.integer); };

    bool getSwitch1()   { return(this->BMData.switch1); };
    bool getSwitch2()   { return(this->BMData.switch2); };
    bool getSwitch3()   { return(this->BMData.switch3); };
    bool getSwitch4()   { return(this->BMData.switch4); };
    bool getSwitch5()   { return(this->BMData.switch5); };
    bool getSwitch6()   { return(this->BMData.switch6); };


    /**
     * @brief Get Millis
     * @return Millis
    */
    uint32_t getMillis() { return *this->millis; }

};

#endif