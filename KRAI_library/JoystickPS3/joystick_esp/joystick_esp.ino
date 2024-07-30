/*
 * 
 * 
 * 
 */

#include <Ps3Controller.h>
#define RX 3
#define TX 1

// Deklarasi variabel tombol analog
signed char LX, LY, RX_, RY;
unsigned char aL2, aR2;
 
// Deklarasi varibel data yang dikirim
unsigned char button;
unsigned char RL;
unsigned int button_click;
unsigned int RL_click;

/*********************************************************************************************/
/**                                                                                         **/
/** FUNGSI PENGIRIMAN DATA                                                                  **/
/** -   Data yang akan dikirim adalah paket data 8-bit dengan urutan sebagai berikut        **/
/** |------|------|--------|----|--------------|----------|----|----|----|----|----|----|   **/
/** | 0x88 | 0x08 | button | RL | button_click | RL_click | R2 | L2 | RX | RY | LX | LY |   **/
/** |------|------|--------|----|--------------|----------|----|----|----|----|----|----|   **/
/**                                                                                         **/
/*********************************************************************************************/
 
void sendJoystickData() {
  Serial.write(0x88);
  Serial.write(0x08);
  Serial.write(button);
  Serial.write(RL);
  Serial.write(button_click);
  Serial.write(RL_click);
  Serial.write(aR2);
  Serial.write(aL2);
  Serial.write(RX_);
  Serial.write(RY);
  Serial.write(LX);
  Serial.write(LY);
  Serial.write(0x18);
  
//  Serial.print(millis());
//  Serial.print("\t");
  Serial.print(button);
  Serial.print("\t");
  Serial.print(RL);
  Serial.print("\t");
  Serial.print(button_click);
  Serial.print("\t");
  Serial.print(RL_click);
  Serial.print("\t");
  Serial.print(aR2);
  Serial.print("\t");
  Serial.print(aL2);
  Serial.print("\t");
  Serial.print(RX_);
  Serial.print("\t");
  Serial.print(RY);
  Serial.print("\t");
  Serial.print(LX);
  Serial.print("\t");
  Serial.println(LY);
}

void notify()
{
  button = 0;
  RL = 0;
  button_click = 0;
  RL_click = 0;
  LX = 0;
  LY = 0;
  RX_ = 0;
  RY = 0;
  aL2 = 0;
  aR2 = 0;

  if (!Ps3.isConnected()) {
    Serial.write(0x88);
    Serial.write(0x09);
  }
  else {
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.data.button.triangle )
      button = button + (0x1 << 0);
    if( Ps3.data.button.circle )
      button = button + (0x1 << 1);
    if( Ps3.data.button.cross )
      button = button + (0x1 << 2);
    if( Ps3.data.button.square )
      button = button + (0x1 << 3);
      
    //--------------- Digital D-pad button events --------------
    if( Ps3.data.button.up )
      button = button + (0x1 << 4);
    if( Ps3.data.button.right )
      button = button + (0x1 << 5);
    if( Ps3.data.button.down )
      button = button + (0x1 << 6);
    if( Ps3.data.button.left )
      button = button + (0x1 << 7);
      
    //------------- Digital shoulder button events -------------
    if( Ps3.data.button.r1 )
      RL = RL + (0x1 << 0);
    if( Ps3.data.button.r3 )
      RL = RL + (0x1 << 1);
    if( Ps3.data.button.l1 )
      RL = RL + (0x1 << 2);
    if( Ps3.data.button.l3 )
      RL = RL + (0x1 << 3);
    if( Ps3.data.button.start )
      RL = RL + (0x1 << 4);
    if( Ps3.data.button.select )
      RL = RL + (0x1 << 5);
    if( Ps3.data.button.ps )
      RL = RL + (0x1 << 6);
    
    //--- Digital cross/square/triangle/circle button events ---
    if( Ps3.event.button_down.triangle )
      button_click = button_click + (0x1 << 0);
    if( Ps3.event.button_down.circle )
      button_click = button_click + (0x1 << 1);
    if( Ps3.event.button_down.cross )
      button_click = button_click + (0x1 << 2);
    if( Ps3.event.button_down.square )
      button_click = button_click + (0x1 << 3);
      
    //--------------- Digital D-pad button events --------------
    if( Ps3.event.button_down.up )
      button_click = button_click + (0x1 << 4);
    if( Ps3.event.button_down.right )
      button_click = button_click + (0x1 << 5);
    if( Ps3.event.button_down.down )
      button_click = button_click + (0x1 << 6);
    if( Ps3.event.button_down.left )
      button_click = button_click + (0x1 << 7);
    
    // if( Ps3.event.button_down.l2 )
    //   ps_msg.l2 = 1;
    // if( Ps3.event.button_down.r2 )
    //   ps_msg.r2 = 1;

    //------------- Digital shoulder button events -------------
    if( Ps3.event.button_down.r1 )
      RL_click = RL_click + (0x1 << 0);
    if( Ps3.event.button_down.r3 )
      RL_click = RL_click + (0x1 << 1);
    if( Ps3.event.button_down.l1 )
      RL_click = RL_click + (0x1 << 2);
    if( Ps3.event.button_down.l3 )
      RL_click = RL_click + (0x1 << 3);
    if( Ps3.event.button_down.start )
      RL_click = RL_click + (0x1 << 4);
    if( Ps3.event.button_down.select )
      RL_click = RL_click + (0x1 << 5);
    if( Ps3.event.button_down.ps )
      RL_click = RL_click + (0x1 << 6);

    //---------------- Analog stick value events ---------------
//    if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
      LX = Ps3.data.analog.stick.lx;
      LY = Ps3.data.analog.stick.ly;
//    }
//    if( abs(Ps3.ev ent.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
      RX_ = Ps3.data.analog.stick.rx;
      RY = Ps3.data.analog.stick.ry;
//    }

    //---------------- Analog D-Pad ---------------
    if( abs(Ps3.data.button.l2) ){
      aL2 = Ps3.data.analog.button.l2;
    }
    if( abs(Ps3.data.button.r2) ){
      aR2 = Ps3.data.analog.button.r2;
    }

    sendJoystickData();
  }
}

void setup()
{
  //komunikasi ke MIKON via UART
  Serial.begin(230400, SERIAL_8N1, RX, TX);

  //komunikasi ke esp32
  Ps3.attach(notify);
  Ps3.begin("00:1B:10:00:2A:EB");   // Ubah MAC Address sesuai dengan PS3 Controller
}

void loop()
{
  delay(100);
}
