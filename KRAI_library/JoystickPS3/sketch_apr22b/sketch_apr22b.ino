/*
 * 
 * 
 * 
 */

#include <ros.h>
#include <joystick_msg/PS3joystick.h>
#include <Ps3Controller.h>

ros::NodeHandle nh;

joystick_msg::PS3joystick ps_msg;
ros::Publisher ps_pub("ps_pub", &ps_msg);

void notify()
{
  //--- Digital cross/square/triangle/circle button events ---
  if( Ps3.event.button_down.cross )
    ps_msg.cross = 1;
  if( Ps3.event.button_up.cross )
    ps_msg.cross = 0;
  if( Ps3.event.button_down.square )
    ps_msg.square = 1;
  if( Ps3.event.button_up.square )
    ps_msg.square = 0;
  if( Ps3.event.button_down.triangle )
    ps_msg.triangle = 1;
  if( Ps3.event.button_up.triangle )
    ps_msg.triangle = 0;
  if( Ps3.event.button_down.circle )
    ps_msg.circle = 1;
  if( Ps3.event.button_up.circle )
    ps_msg.circle = 0;
    
  //--------------- Digital D-pad button events --------------
  if( Ps3.event.button_down.up )
    ps_msg.up = 1;
  if( Ps3.event.button_up.up )
    ps_msg.up = 0;
  if( Ps3.event.button_down.right )
    ps_msg.right = 1;
  if( Ps3.event.button_up.right )
    ps_msg.right = 0;
  if( Ps3.event.button_down.down )
    ps_msg.down = 1;
  if( Ps3.event.button_up.down )
    ps_msg.down = 0;
  if( Ps3.event.button_down.left )
    ps_msg.left = 1;
  if( Ps3.event.button_up.left )
    ps_msg.left = 0;
    
  //------------- Digital shoulder button events -------------
  if( Ps3.event.button_down.l1 )
    ps_msg.l1 = 1;
  if( Ps3.event.button_up.l1 )
    ps_msg.l1 = 0;
  if( Ps3.event.button_down.r1 )
    ps_msg.r1 = 1;
  if( Ps3.event.button_up.r1 )
    ps_msg.r1 = 0;

  //-------------- Digital trigger button events -------------
  if( Ps3.event.button_down.l2 )
    ps_msg.l2 = 1;
  if( Ps3.event.button_up.l2 )
    ps_msg.l2 = 0;
  if( Ps3.event.button_down.r2 )
    ps_msg.r2 = 1;
  if( Ps3.event.button_up.r2 )
    ps_msg.r2 = 0;

  //--------------- Digital stick button events --------------
  if( Ps3.event.button_down.l3 )
    ps_msg.l3 = 1;
  if( Ps3.event.button_up.l3 )
    ps_msg.l3 = 0;
  if( Ps3.event.button_down.r3 )
    ps_msg.r3 = 1;
  if( Ps3.event.button_up.r3 )
    ps_msg.r3 = 0;

  //---------- Digital select/start/ps button events ---------
  if( Ps3.event.button_down.select )
    ps_msg.select = 1;
  if( Ps3.event.button_up.select )
    ps_msg.select = 0;
  if( Ps3.event.button_down.start )
    ps_msg.start = 1;
  if( Ps3.event.button_up.start )
    ps_msg.start = 0;
  if( Ps3.event.button_down.ps )
    ps_msg.ps = 1;
  if( Ps3.event.button_up.ps )
    ps_msg.ps = 0;

  //---------------- Analog stick value events ---------------
//  if( abs(Ps3.event.analog_changed.stick.lx) + abs(Ps3.event.analog_changed.stick.ly) > 2 ){
//    ps_msg.lX = Ps3.data.analog.stick.lx;
//    ps_msg.lY = Ps3.data.analog.stick.ly;
//  }
//  if( abs(Ps3.event.analog_changed.stick.rx) + abs(Ps3.event.analog_changed.stick.ry) > 2 ){
//    ps_msg.rX = Ps3.data.analog.stick.rx;
//    ps_msg.rY = Ps3.data.analog.stick.ry;
//  }
}

void onConnect(){
  ps_msg.isConnected = "Connected";
  Serial.println("Connected.");
}

void setup()
{
  nh.initNode();
  Serial.begin(115200);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("02:D2:92:5E:63:25");
  nh.advertise(ps_pub);
}

void loop()
{
  ps_pub.publish( &ps_msg );
  nh.spinOnce();
  delay(100);
}
