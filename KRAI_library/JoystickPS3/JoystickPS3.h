#ifndef _JOYSTICKPS3_H_
#define _JOYSTICKPS3_H_

#include "mbed.h"

// CONSTANTS
#ifndef PI
#define PI 3.14159265358979f

#endif
#define VEL_TRANSLASI_HIGH 1.2f
#define VEL_TRANSLASI_LOW 0.6f
#define VEL_ROTASI_HIGH (2*PI/5)
#define VEL_ROTASI_LOW (PI/5)
#define JOYSTICK_DEFAULT_BAUD 230400

class JoystickPS3
{
private:
    BufferedSerial _serial;
    unsigned char buf;

    // unsigned char getChar();
    int data_get;

    // Deklarasi variabel tombol analog
    int LX, LY, RX, RY, R2, L2;

    unsigned char button;
    unsigned char RL;
    unsigned char button_click;
    unsigned char RL_click;

    bool is_print;

    // Deklarasi variabel tombol joystik
    bool kiri, kanan, atas, bawah;
    bool segitiga, lingkaran, kotak, silang;
    bool L1, R1, L3, R3, START, SELECT, PS;

    bool kiri_click, kanan_click, atas_click, bawah_click;
    bool segitiga_click, lingkaran_click, kotak_click, silang_click;
    bool L1_click, R1_click, L3_click, R3_click, R2_click, L2_click;
    bool R2sebelum, L2sebelum;
    bool START_click, SELECT_click, PS_click;

    float vx_cmd_ins;
    float vy_cmd_ins;
    float w_cmd_ins;

    uint8_t send_spi;
    uint32_t sampling_spi;

    bool isBOHHmoving;

    uint32_t millis = 0;


public:
    // Constructor
    JoystickPS3(PinName tx, PinName rx) : _serial(tx, rx) {}

    unsigned char getChar();

    // Setter for send_spi
    void set_send_spi(uint8_t data);

    // Setter for sampling_spi
    void set_sampling_spi(uint32_t data);

    void updateMillis(uint32_t data) {this->millis = data;}
    int getMillis() { return this->millis; }

    // Setup joystick connection
    void setup();

    /*********************************************************************************************/
    /**                                                                                         **/
    /** FUNGSI PEMBACAAN DATA                                                                   **/
    /** -   Data yang diterima dari Serial Arduino berbentuk 8-bit                              **/
    /** -   Data yang diterima diolah menjadi boolean / 1-bit untuk data tombol button dan RL   **/
    /**     karena data yang digunakan adalah 1-bit (true/false)                                **/
    /** -   Untuk analog data yang diterima tidak diolah karena rentang data yang dikirimkan    **/
    /**     memiliki rentang 0-255 / 8-bit, dan data yang akan digunakan adalah data 8-bit      **/
    /**                                                                                         **/
    /**         |------|-------|-------|------|-------|--------|-----------|----------|         **/
    /** Bit Ke  |   7  |   6   |   5   |   4  |   3   |    2   |      1    |     0    |         **/
    /**         |------|-------|-------|------|-------|--------|-----------|----------|         **/
    /** Data    | kiri | bawah | kanan | atas | kotak | silang | lingkaran | segitiga |         **/
    /**         |------|-------|-------|------|-------|--------|-----------|----------|         **/
    /**                                                                                         **/
    /** -   Penggabungan data R1, R2, L1, L2, R3, L3, START, dan SELECT disimpan dalam          **/
    /**     variabel "RL"                                                                       **/
    /** -   Urutan data pada variabel "RL" dan "RL_click" adalah                                **/
    /**     sebagai berikut                                                                     **/
    /**                                                                                         **/
    /**         |----|--------|-------|----|----|----|----|                                     **/
    /** Bit Ke  |  6 |    5   |   4   |  3 |  2 |  1 |  0 |                                     **/
    /**         |----|--------|-------|----|----|----|----|                                     **/
    /** Data    | PS | SELECT | START | L3 | L1 | R3 | R1 |                                     **/
    /**         |----|--------|-------|----|----|----|----|                                     **/
    /**                                                                                         **/
    /*********************************************************************************************/

    void olah_data();

    /*********************************************************************************************/
    /**                                                                                         **/
    /** FUNGSI IDLE                                                                             **/
    /** -   Fungsi dijalankan saat Arduino mengirimkan data yang merupakan                      **/
    /**     kondisi PS3 Disconnected                                                            **/
    /** -   Fungsi membuat semua data joystik bernilai 0                                        **/
    /**                                                                                         **/
    /*********************************************************************************************/

    void idle();

    /*********************************************************************************************/
    /**                                                                                         **/
    /** FUNGSI PEMBACAAN DATA                                                                   **/
    /** -   Fungsi pembacaan data yang dikirim dari arduino                                     **/
    /** -   Data yang dikirim dari arduino merupakan paket data dengan format pengiriman        **/
    /**                                                                                         **/
    /** |------|------|--------|----|--------------|----------|----|----|----|----|----|----|   **/
    /** | 0x88 | 0x08 | button | RL | button_click | RL_click | R2 | L2 | RX | RY | LX | LY |   **/
    /** |------|------|--------|----|--------------|----------|----|----|----|----|----|----|   **/
    /**                                                                                         **/
    /** |------|------|                                                                         **/
    /** | 0x88 | 0x09 |                                                                         **/
    /** |------|------|                                                                         **/
    /**                                                                                         **/
    /** -   Jika urutan data yang diterima seperti tabel diatas, maka data tersebut akan        **/
    /**     diolah untuk input ke aktuator                                                      **/
    /**                                                                                         **/
    /*********************************************************************************************/

    bool baca_data();

    // Set true of readable

    bool readable();

    // L2 pressed -> send 1 -> automatic
    bool getMode();

    // R2 pressed -> send 1 -> automatic
    bool parallelParkMode();

    // kotak pressed
    bool offsetFromPoleMode();

    // Read stick state and modify vc_cmd, vy_cmd, and w_cmd accordingly

    void stickState(float *vx_cmd, float *vy_cmd, float *w_cmd);

    // Send SPI Stick State to main program

    void sendSPIStickState(uint8_t *data_send_spi, uint32_t *data_sampling_spi);
    // , bool *digL, bool *digR

    bool getSelect();

    bool getStart();
    // Getter for all button state
    bool getButtonUp();

    bool getButtonDown();

    bool getButtonRight();

    bool getButtonLeft();

    bool getSegitiga();

    bool getLingkaran();

    bool getSilang();

    bool getKotak();

    bool getR1();

    bool getR2();

    bool getL1();

    bool getL2();

    int getRX();
    int getRY();
    int getLX();
    int getLY();
};
#endif