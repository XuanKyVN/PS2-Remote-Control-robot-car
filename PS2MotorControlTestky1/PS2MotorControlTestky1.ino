
#include <PS2X_lib.h>  //for v1.6

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/

//  ESP32 pin
// https://github.com/espressif/arduino-esp32/blob/master/docs/esp32_pinmap.png

#define PS2_DAT        42  //MISO  19
#define PS2_CMD        40  //MOSI  23
#define PS2_SEL         41  //SS     5
#define PS2_CLK        39  //SLK   18

/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
#define pressures   false
#define rumble      false

PS2X ps2x; // create PS2 Controller Class

//right now, the library does NOT support hot pluggable controllers, meaning
//you must always either restart your Arduino after you connect the controller,
//or call config_gamepad(pins) again after connecting the controller.

int error = -1;
byte type = 0;
byte vibrate = 0;
int tryNum = 1;

//-----------------------------------------------
bool fwd_status;
bool stop_status;
bool bwd_status;
bool left_status;
bool right_status;


//---------------------------
//--------------Motor variable-----------------
enum {
  //enDisable=1,
  //enEnable,
  enSTOP,
  enRUN,
  enBACK,
  enLEFT,
  enRIGHT,
  enUPLEFT,
  enUPRIGHT,
  enDOWNLEFT,
  enDOWNRIGHT,
  enSpinLEFT,
  enSpinRIGHT
} enCarState;
int CarSpeedControl = 0;  // Range Speed 30 to 220 
//int g_CarState = enDisable;  
int g_CarState = enSTOP;  
//--------------------------------------
//----------------------------------------REMOTE-------------------------------
void config_remoteps2(){
   //added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  while (error != 0) {
    delay(1000);// 1 second wait
    //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
    Serial.print("#try config ");
    Serial.println(tryNum);
    tryNum ++;
  }

  Serial.println(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch(type) {
    case 0:
      Serial.println(" Unknown Controller type found ");
      break;
    case 1:
      Serial.println(" DualShock Controller found ");
      break;
    case 2:
      Serial.println(" GuitarHero Controller found ");
      break;
	  case 3:
      Serial.println(" Wireless Sony DualShock Controller found ");
      break;
   }
}

//---------------------------------------
// Checking Signal from remote
void receive_remote_ps2(){
 
  if(type == 1){ //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    // Function set car status
    //set_status_of_car(bool fwd,bool bwd, bool left, bool right, bool stopst) 
    /*
    //will be TRUE as long as button is pressed
    if(ps2x.Button(PSB_START)){
     
      g_CarState=enEnable;
       Serial.println("Start Enable car");
    }
    */
    if(ps2x.Button(PSB_SELECT)){
      
      g_CarState=enSTOP;
      set_status_of_car(0,0,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
      CarSpeedControl = 0;  //minimum speed
      Serial.println("Disable Car");
    }
    
        if(ps2x.Button(PSB_PAD_UP)) {
          
          g_CarState = enRUN;
          set_status_of_car(1,0,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          Serial.println("move fwd: "  + String(CarSpeedControl));
          
        }
        if(fwd_status && ps2x.Button(PSB_CIRCLE)&&CarSpeedControl>30){
          
          g_CarState = enUPRIGHT;
          set_status_of_car(1,0,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          Serial.println("turn right fwd , Left motor speed: " + String(CarSpeedControl+30) + ", Right Motor Speed: "+ String(CarSpeedControl-30));
        }
    
        if(fwd_status && ps2x.Button(PSB_SQUARE)&&CarSpeedControl>30){
          
          g_CarState = enUPLEFT;
          set_status_of_car(1,0,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          Serial.println("Turn left Fwd, Left motor speed: " + String(CarSpeedControl-30) + ", Right Motor Speed: "+ String(CarSpeedControl+30));
        }
    //------------------------------------BWD----------------------
        
        if(ps2x.Button(PSB_PAD_DOWN)) {
          
          g_CarState = enBACK;
          set_status_of_car(0,1,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          Serial.println("move bwd: " + String(CarSpeedControl));
        }
        if(bwd_status &&ps2x.Button(PSB_CIRCLE) && CarSpeedControl>30){
          
          g_CarState = enDOWNRIGHT;
          set_status_of_car(0,1,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          Serial.println("turn right bwd, Left motor speed: " + String(CarSpeedControl+30) + ", Right Motor Speed: "+ String(CarSpeedControl-30));
        }
    
        if(bwd_status && ps2x.Button(PSB_SQUARE)&&CarSpeedControl>30){
          
          g_CarState = enDOWNLEFT;
          set_status_of_car(0,1,0,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          Serial.println("Turn left bwd, Left motor speed: " + String(CarSpeedControl-30) + ", Right Motor Speed: "+ String(CarSpeedControl+30));
        }
    
    
    //--------------------------------------left and right----------------------
    
     if(ps2x.Button(PSB_PAD_LEFT)){
         
          g_CarState = enLEFT;
          set_status_of_car(0,0,1,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
           Serial.println("Turn left, right motor run, left motor off: " + String(CarSpeedControl));
        }
    
     if(left_status && ps2x.ButtonPressed(PSB_R2)){  // SPIN LEFT
          
          set_status_of_car(0,0,1,0,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          g_CarState= enSpinRIGHT;
          Serial.println("Turn left, right motor run reverse, left motor run forward: " + String(CarSpeedControl));
          
        }
    //---------   Right------------
    
     if(ps2x.Button(PSB_PAD_RIGHT)){
          
          set_status_of_car(0,0,0,1,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          g_CarState = enRIGHT;
          Serial.println("Turn right, Left motor run, right motor off: " + String(CarSpeedControl));
        }
    
     if(right_status && ps2x.Button(PSB_R2)){ // Spin right
          
          set_status_of_car(0,0,0,1,0); //(bool fwd,bool bwd, bool left, bool right, bool stopst)
          g_CarState= enSpinRIGHT;
          Serial.println("Turn right, left motor run fwd, left motor run backward: " + String(CarSpeedControl));
        }
    
//     if (ps2x.Button(PSB_L2)){  // Stop is L3
//      Serial.println("Car Stop");
//      g_CarState = enSTOP; // Break;   // speed =0, motor stop   
//     }
    //△□○×-----------------------------------SPEED ADJUST----------------
    if(ps2x.ButtonPressed(PSB_TRIANGLE))  {             
      
        CarSpeedControl += 10;
        if (CarSpeedControl > 230)
          {
            CarSpeedControl = 230;
          }

       if (CarSpeedControl < 40)
          {
            CarSpeedControl = 40;
          }

          Serial.println("△ Increase Speed: " + String(CarSpeedControl));
    }

    if(ps2x.ButtonPressed(PSB_CROSS)) {              
     
        CarSpeedControl -=10;
        if (CarSpeedControl < 40)
          {
            CarSpeedControl = 40;
          }
      Serial.println("X Decrease Speed: " + String(CarSpeedControl));
    }
     //-------------------------------
 //  FOR ANALOGE CONTROL Car by JOYSTIC RIGHT
   //-------------------------------------------------
    
  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1))
  {
    //print stick values if either is TRUE
    int X1 = ps2x.Analog(PSS_RX);
    int Y1 = ps2x.Analog(PSS_RY);

    if (Y1 < 5 && X1 > 80 && X1 < 180)         //UP
    {
      g_CarState = enRUN;
      Serial.println("Move FWD: " + String(CarSpeedControl));
    }
    else if (Y1 > 230 && X1 > 80 && X1 < 180) //DOWN
    {
      g_CarState = enBACK;
      Serial.println("Move BWD: " + String(CarSpeedControl));
    }
    else if (X1 < 5 && Y1 > 80 && Y1 < 180)   //LEFT
    {
      g_CarState = enLEFT;
      Serial.println("Move LEFT: " + String(CarSpeedControl));
    }
    else if (Y1 > 80 && Y1 < 180 && X1 > 230) //RIGHT
    {
      g_CarState = enRIGHT;
      Serial.println("Move Right: " + String(CarSpeedControl));
    }
    else if (Y1 <= 80 && X1 <= 80 &&CarSpeedControl>30)           //upper left
    {
      g_CarState = enUPLEFT;
      Serial.println("turn right fwd , Left motor speed: " + String(CarSpeedControl-30) + ", Right Motor Speed: "+ String(CarSpeedControl+30));
    }
    else if (Y1 <= 80 && X1 >= 180 &&CarSpeedControl>30)          //upper right
    {
      g_CarState = enUPRIGHT;
     Serial.println("turn right fwd , Left motor speed: " + String(CarSpeedControl+30) + ", Right Motor Speed: "+ String(CarSpeedControl-30));
    }
    else if (X1 <= 80 && Y1 >= 180 &&CarSpeedControl>30)          //left lower
    {
      g_CarState = enDOWNLEFT;
       Serial.println("Turn left bwd, Left motor speed: " + String(CarSpeedControl-30) + ", Right Motor Speed: "+ String(CarSpeedControl+30));
    }
    else if (Y1 >= 180 && X1 >= 180 &&CarSpeedControl>30)        //right lower
    {
      g_CarState = enDOWNRIGHT;
       Serial.println("turn right bwd, Left motor speed: " + String(CarSpeedControl+30) + ", Right Motor Speed: "+ String(CarSpeedControl-30));


    }
    else                                         //stop
    {
      g_CarState = enSTOP;
      CarSpeedControl =0;
      Serial.println("STOP CAR: " + String(CarSpeedControl));
      set_status_of_car(0,0,0,0,1); //(bool fwd,bool bwd, bool left, bool right, bool stopst)

    }

    }// Button R2 press
}
}



//-------------------------------------------------------------
// MOTOR CONTROL
//------------------------------------------------------------

#include "BTS7960.h"
// variable to receive and control motor
//int motorspeed_cmd, direct_cmd; bool motor_enb_cmd;
// direct_cmd 0 stop, 1 fwd,2 bwd,3 left,4 right
//------------------------------------------------------------------------
// Right Motor working normal  PIN of ESP32
const uint8_t L_EN_2 = 4;  //Pin GPIO5 ESP32 ,on Arduino Mega 7
const uint8_t LPWM_2 = 5; //Pin GPIO4 ESP32,on Arduino Mega 11

const uint8_t R_EN_2 = 6;  //Pin GPIO4 ESP32,on Arduino Mega 10
const uint8_t RPWM_2 = 7; //Pin GPIO4 ESP32,on Arduino Mega 9

//// Left Motor testing
const uint8_t L_EN_1 = 8;  //ESP32 GPIO8 ,on Arduino Mega 5
const uint8_t LPWM_1 = 3;  // ESP32 GPIO9 on Arduino Mega 6  // Reverse

const uint8_t R_EN_1 = 46;  //ESP32 GPIO46 on Arduino Mega 4
const uint8_t RPWM_1 = 9; // ESP32 GPIO3 on Arduino Mega 8  //FWD

BTS7960 LeftMotor(L_EN_1,R_EN_1, LPWM_1, RPWM_1);
BTS7960 RightMotor(L_EN_2,R_EN_2, LPWM_2, RPWM_2);
//---------------------------------------------------




//---------------------------------------------------MOTOR----------

void set_status_of_car(bool fwd,bool bwd, bool left, bool right, bool stopst){
fwd_status=fwd;
stop_status=stopst;
bwd_status=bwd;
left_status=left;
right_status=right;
}



void enable_robot(){  // enable car
   LeftMotor.Enable();
   RightMotor.Enable();
}
void disable_robot(){ // Disable car
  LeftMotor.Stop();
  RightMotor.Stop();
  LeftMotor.Disable();
  RightMotor.Disable();
}

void fwd() // go forward
{
enable_robot();
LeftMotor.TurnRight(CarSpeedControl);    
RightMotor.TurnRight(CarSpeedControl);
Serial.println("HI I AM HERE");
}


void brake()
{
  LeftMotor.Stop();
  RightMotor.Stop();
}


void left() // car turn left while right motor is off, 
{
  LeftMotor.TurnRight(CarSpeedControl);    
  RightMotor.Stop();
}


void upleft() // car move fwd but turn left with different speed
{
  LeftMotor.TurnRight(CarSpeedControl-30);     // speed >= 30  ;  Speed <=220;  
  RightMotor.TurnRight(CarSpeedControl+30);
}


void downleft()  // car move bwd but turn left with different speed
{
  
  LeftMotor.TurnLeft(CarSpeedControl-30);    
  RightMotor.TurnLeft(CarSpeedControl+30);
}


void spin_left()
{
  LeftMotor.TurnLeft(CarSpeedControl);    
  RightMotor.TurnRight(CarSpeedControl);
}


void right()
{
  LeftMotor.Stop();  
  RightMotor.TurnRight(CarSpeedControl);
}

  

void upright() // car move fwd but diff speed to turn right
{
  LeftMotor.TurnRight(CarSpeedControl+30);    
  RightMotor.TurnRight(CarSpeedControl-30);
}
   

void downright() // car move backward but turn right
{
  LeftMotor.TurnLeft(CarSpeedControl+30);    
  RightMotor.TurnLeft(CarSpeedControl-30);
}



void spin_right()// Left motor run Fwd, right motor run Bwd
{
  
  LeftMotor.TurnRight(CarSpeedControl);    
  RightMotor.TurnLeft(CarSpeedControl);
}


void back()
{
  LeftMotor.TurnLeft(CarSpeedControl);    
  RightMotor.TurnLeft(CarSpeedControl);
}


//-------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  // 115200
  Serial.begin(115200);
  config_remoteps2();



  
}

void loop() {
  // put your main code here, to run repeatedly:

enable_robot();
//---------------------------------
receive_remote_ps2();   // Read remote 
switch (g_CarState)  // Control Motor
  {
    //case enDisable: disable_robot(); break;
    //case enEnable: enable_robot();break;
    case enSTOP: brake(); break;
    case enRUN: fwd();  break;
    case enLEFT: left();  break;
    case enRIGHT: right(); break;
    case enBACK: back(); break;
    case enUPLEFT: upleft();  break;
    case enUPRIGHT: upright(); break;
    case enDOWNLEFT: downleft();  break;
    case enDOWNRIGHT: downright(); break;
    case enSpinLEFT: spin_left(); break;
    case enSpinRIGHT: spin_right(); break;
    default: break;
  }
//-----------------------------------


delay(100);


}
