
//Include libraries for IR remote, i2c, and OLED
#include<IRremote.h>
#include <Wire.h>
//#include "U8glib.h"

//identify correct OLED class
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);

int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;

//*********************************************************************
//CONNECT RIGHT MOTOR TO M1 PORT
//CONNECT LEFT MOTOR TO M2 PORT

//states for the motor
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT_TURN 3
#define RIGHT_TURN 4

# define MOTOR_LEFT_A 5
# define MOTOR_LEFT_B 6
# define MOTOR_RIGHT_A 10
# define MOTOR_RIGHT_B 9


//*********************************************************************
void setup() {

  //IR SETUP---------------------------------
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  //u8g.firstPage();

  //u8g.setFont(u8g_font_04b_24);
  //u8g.setFont(u8g_font_unifont);

  //Motor SETUP--------------------------------
  //this has to go first otherwise motors will go crazy
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  //make sure motors are off
  setMotorState(0, 0, 0);

}
//*********************************************************************

void loop() {
    int selectedState = readIR();

    //Serial.println(selectedState);

    if (selectedState == 1 || 2) {
       setMotorState(selectedState, 150, 150);
       delay(200);
    }
    if  (selectedState == 3 || 4) {
       setMotorState(selectedState, 60, 60);
       delay(150);
    }
   
    setMotorState(0,0,0);

}


//*********************************************************************
int readIR() {

  int state = 0;
  if (irrecv.decode(&results))
  {
    if (results.value == 0xE0E0807F)
    {
      Serial.println("FORWARD");
      state = 1;
    }

    else if (results.value == 0xE0E040BF)
    {
      Serial.println("BACKWARD");
      state = 2;
    }

    else if (results.value == 0xE0E020DF)
    {
      Serial.println("LEFT");
      state = 4;
    }

    else if (results.value == 0xE0E010EF)
    {
      Serial.println("RIGHT");
      state = 3;

    }

    else {
      state=0;
    }

    irrecv.resume();
  }

  return state;
}

//*********************************************************************
byte setMotorState(byte selectedMotorState, int speedM1, int speedM2)
{
  byte motorState = selectedMotorState;
  switch (selectedMotorState) {
    //--------------------------------------
    case 0://Motors stop (0,0)
      //0 = 0
      analogWrite(MOTOR_LEFT_A, 0);
      analogWrite(MOTOR_LEFT_B, 0);
      analogWrite(MOTOR_RIGHT_A, 0);
      analogWrite(MOTOR_RIGHT_B, 0);
      break;
    //--------------------------------------
    case 1://forward (+, +)
      //1
      analogWrite(MOTOR_LEFT_A, speedM1);
      analogWrite(MOTOR_LEFT_B, 0);
      analogWrite(MOTOR_RIGHT_A, speedM2);
      analogWrite(MOTOR_RIGHT_B, 0);
      break;
    //--------------------------------------
    case 2://backward (-,-)
      //2
      analogWrite(MOTOR_LEFT_A, 0);
      analogWrite(MOTOR_LEFT_B, speedM1);
      analogWrite(MOTOR_RIGHT_A, 0);
      analogWrite(MOTOR_RIGHT_B, speedM2);
      break;
    //--------------------------------------
    case 3://left with both motors
      // 3 (+, -)
      analogWrite(MOTOR_LEFT_A, speedM1);
      analogWrite(MOTOR_LEFT_B, 0);
      analogWrite(MOTOR_RIGHT_A, 0);
      analogWrite(MOTOR_RIGHT_B, speedM2);
      break;
    //--------------------------------------
    case 4://right with both motors
      // 4 (-, +)
      analogWrite(MOTOR_LEFT_A, 0);
      analogWrite(MOTOR_LEFT_B, speedM1);
      analogWrite(MOTOR_RIGHT_A, speedM2);
      analogWrite(MOTOR_RIGHT_B, 0);
      break;
    //--------------------------------------

    case 5://turn left with right motor
      // 5 (+, 0)
      analogWrite(MOTOR_LEFT_A, speedM1);
      analogWrite(MOTOR_LEFT_B, 0);
      analogWrite(MOTOR_RIGHT_A, 0);
      analogWrite(MOTOR_RIGHT_B, 0);
      break;
    //--------------------------------------
    case 6://turn right with right motor
      // 6 (-, 0)
      analogWrite(MOTOR_LEFT_A, 0);
      analogWrite(MOTOR_LEFT_B, speedM1);
      analogWrite(MOTOR_RIGHT_A, 0);
      analogWrite(MOTOR_RIGHT_B, 0);
      break;
    //--------------------------------------

    case 7://turn right with left motor
      //7 (0, +)
      analogWrite(MOTOR_LEFT_A, 0);
      analogWrite(MOTOR_LEFT_B, 0);
      analogWrite(MOTOR_RIGHT_A, speedM2);
      analogWrite(MOTOR_RIGHT_B, 0);
      break;
    //--------------------------------------

    case 8:////turn left w left motor
      // 8 (0, -)
      analogWrite(MOTOR_LEFT_A, 0);
      analogWrite(MOTOR_LEFT_B, 0);
      analogWrite(MOTOR_RIGHT_A, 0);
      analogWrite(MOTOR_RIGHT_B, speedM2);
      break;
    //--------------------------------------

    default:
      break;
  }
  return motorState;
}
