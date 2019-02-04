//V4.0 Zumi
//Select Arduino UNO under board

#include <Arduino.h>

//The following can be found on
//the Arduino Library Manager

//Library MPU6050 Library
#include <MPU6050_tockn.h>
//I2C Library
#include <Wire.h>
//Infrared Remote Library
#include <IRremote.h>

//Library for the OLED
#include "U8glib.h"
//I am using the I2C one so select the following class
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);

//define max of the screen pixels
#define ROWMAX 128
#define COLMAX 64
//zumi eyes
# define eyeSpeed 2
//in pixels
# define eyeWidth 10

//A7 =D21 is multiplex out
# define MUX_OUTPUT 21

//multiplexer select pins
//A1=S0, A2=S1, A3=S2
#define MUX_S0 15
#define MUX_S1 16
#define MUX_S2 17

//front headlights and rear lights
# define FRONT_LEFT_LED 11
# define FRONT_RIGHT_LED 12
# define BACK_LEFT_LED  8
# define BACK_RIGHT_LED  7

//BUZZER is on pin A0 = D14
# define BUZZER_PIN 14

//Battery is connected on pin A6 = D20
#define BATT_LVL_PIN 20

//IR receiver is on pin DIG4
#define IR_REMREC_PIN 4
IRrecv irrecv(IR_REMREC_PIN);
decode_results results;
IRsend irsend;


//**CHANGED
# define IR_EMIT_PIN 3
//IR Emitters control pin
//for the transistors is on DIG 3.

//**CHANGED
//A3906 Motor Controller is on pins
# define MOTOR_LEFT_A 5
# define MOTOR_LEFT_B 6
# define MOTOR_RIGHT_A 10
# define MOTOR_RIGHT_B 9

//CONNECT RIGHT MOTOR TO M1 PORT
//CONNECT LEFT MOTOR TO M2 PORT

//states for the motor
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT_TURN 3
#define RIGHT_TURN 4

MPU6050 mpu6050(Wire);

double bitsPerVolts = 0;
unsigned long counter = 0;
// the setup routine runs once when you press reset:
void setup() {

  //Motor controller
  //this has to go first otherwise motors will go crazy
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  //make sure motors are off
  setMotorState(0, 0, 0);

  //front and rear lights
  pinMode(FRONT_RIGHT_LED, OUTPUT);
  pinMode(FRONT_LEFT_LED, OUTPUT);
  pinMode(BACK_RIGHT_LED, OUTPUT);
  pinMode(BACK_LEFT_LED, OUTPUT);

  //setup analog pins for multiplexer as output
  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);

  //IR Led's transistor control
  //will control the transistor that controls IR led's
  pinMode(IR_EMIT_PIN, OUTPUT);

  // Start the receiver
  irrecv.enableIRIn();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  //initialize serial communication at 9600 bits per second:
  Serial.begin(115200);

  Wire.begin();
  //set I2C to 400khz
  //Wire.setClock(400000);
  mpu6050.begin();
  //mpu6050.calcGyroOffsets(false);
  mpu6050.setGyroOffsets(-2.70, -0.77, 0.27);

  /*
    X : -2.70
    Y : -0.78
    Z : -0.29

    X : -2.68
    Y : -0.77
    Z : -0.25

    X : -2.19
    Y : -1.17
    Z : 0.27

  */

  //This for an accurate analog Read
  bitsPerVolts = 1023.0 / (readVccMilliVolts() * 0.001);
  
  //not necessary but turning on all lights for testing
  digitalWrite(FRONT_LEFT_LED, HIGH);
  digitalWrite(FRONT_RIGHT_LED, HIGH);
  digitalWrite(BACK_LEFT_LED, HIGH);
  digitalWrite(BACK_RIGHT_LED, HIGH);
  delay(1000);
  excited();

    u8g.firstPage();
  u8g.setFont(u8g_font_04b_24);

}

// the loop routine runs over and over again forever:
void loop() {

  
  u8g.firstPage();

  do {
    counter++;

    if (counter > 0 && counter < 300 / eyeSpeed)
    {
      zumiEyes();
    }
    else if (counter > 300 / eyeSpeed && counter < 500 / eyeSpeed)
    {
      moveEyesLeft();
    }
    else if (counter > 500 / eyeSpeed && counter < 700 / eyeSpeed)
    {
      moveEyesRight();
    }
    else if (counter > 700 / eyeSpeed)
    {
      counter = 0;
    }
    else {

    }
    
  }
  while ( u8g.nextPage() );

  //displayAllIRsensors();


  //   int IRvalueRead = checkIRreceiver();
  //   if(IRvalueRead>0)
  //   {
  //   Serial.println(IRvalueRead, HEX);
  //   }

  //Sending IR messages to other Zumi's
  //    irsend.sendSony(0xa90, 12);
  //   delay(1000);


  //Serial.println(readthisIRSensors(6));

  if (checkBattery() <= 3.1)
  {

    digitalWrite(FRONT_LEFT_LED, HIGH);
    digitalWrite(FRONT_RIGHT_LED, HIGH);
    digitalWrite(BACK_LEFT_LED, HIGH);
    digitalWrite(BACK_RIGHT_LED, HIGH);
  }


  //int rightBIR = averageIRReadings(4, 20, true);
  //int leftBIR = averageIRReadings(2, 20, true);


  //Serial.print(1-1023/leftBIR);
  //Serial.print(",");
  //Serial.print(1-1023/rightBIR);
  //Serial.println("");

  //  Serial.print(leftBIR);
  //  Serial.print(",");
  //  Serial.print(rightBIR);
  //  Serial.print(",");
  //  Serial.print(checkBattery());
  //  Serial.println("");

  // driveForward();

  //setHeading(0, 100);

  //setHeading(0, 0.8);
  //setHeading(0,0.6);
  //setHeading(0,0.5);

  // turnHeading(180, 100);
  //  delay(1000);
  //
  //  turnHeading(0, 100);
  //  delay(1000);
  //
  //  turnHeading(-180, 100);
  //  delay(1000);

  //turnAndFace(90,100);
  //motorTest(25);

 //  motorTest(6);


}

void zumiEyes()
{

  //set the pixel location on the screen
  //(X position, Y position)
  u8g.drawBox(128 / 4, 0, eyeWidth, 64);
  u8g.drawBox(128 / 4 * 3, 0, eyeWidth, 64);

}


void moveEyesLeft()
{

  //set the pixel location on the screen
  //(X position, Y position)
  u8g.drawBox(128 / 4 - 20, 0, eyeWidth, 64);
  u8g.drawBox(128 / 4 * 3 - 20, 0, eyeWidth, 64); 

}

void moveEyesRight()
{
  //set the pixel location on the screen
  //(X position, Y position)
  u8g.drawBox(128 / 4 + 20, 0, eyeWidth, 32);
  u8g.drawBox(128 / 4 * 3 + 20, 0, eyeWidth, 32);

}


void sleeping(){


  
}

//in order to make accurate voltage readings we will us the internal
//voltage reference of 1.1volts
long readVccMilliVolts() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);             // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Convert
  while (bit_is_set(ADCSRA, ADSC));
  result = ADCL;
  result |= ADCH << 8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

double checkBattery()
{
  double batteryVoltage = analogRead(BATT_LVL_PIN) / bitsPerVolts;
  return batteryVoltage;
}

int averageIRReadings(int irIndex, int iterations, bool emitterOn) {
  //start the sum at zero
  int sum = 0;

  for (int i = 0; i < iterations; i++) {
    //add the values
    sum = sum + readthisIRSensors(irIndex, emitterOn);
  }
  return sum / iterations;
}


double detectWhiteLine(int iterations) {

  int valueOn, valueOff, valueDifference;
  double WhiteIntensityLeft, WhiteIntensityRight;
  //take measurements while ir emitter on
  valueOn = averageIRReadings(4, iterations, true);
  //take measurements while ir emitter off
  valueOff = averageIRReadings(4, iterations, false);

  //find difference
  valueDifference = (valueOff - valueOn);

  //combine value on with difference
  //to give you a white intensity score
  WhiteIntensityLeft = valueOn * 100.0 / (valueDifference + 1);

  //take measurements while ir emitter on
  valueOn = averageIRReadings(2, iterations, true);
  //take measurements while ir emitter off
  valueOff = averageIRReadings(2, iterations, false);

  //find difference
  valueDifference = (valueOff - valueOn);

  //combine value on with difference
  //to give you a white intensity score
  WhiteIntensityRight = valueOn * 100.0 / (valueDifference + 1);


  return WhiteIntensityRight - WhiteIntensityLeft;
}

int readthisIRSensors(int indexOfIRSensor, bool emitterOn)
{

  if (emitterOn)
  {
    //analogWrite(IR_EMIT_PIN, 254);
    digitalWrite(IR_EMIT_PIN, HIGH);
    //delay(1);
  }
  else
  {
    digitalWrite(IR_EMIT_PIN, LOW);
  }

  //select different channels on the Multiplexer
  switch (indexOfIRSensor) {
    //--------------------------------------
    case 0:
      //000 = 0
      digitalWrite(15, LOW);
      digitalWrite(16, LOW);
      digitalWrite(17, LOW);
      break;
    //--------------------------------------
    case 1://Y1
      //100 = 4
      digitalWrite(15, HIGH);
      digitalWrite(16, LOW);
      digitalWrite(17, LOW);
      break;
    //--------------------------------------
    case 2://Y2
      //010 = 2
      digitalWrite(15, LOW);
      digitalWrite(16, HIGH);
      digitalWrite(17, LOW);
      break;
    //--------------------------------------
    case 3://Y3
      //110 = 6
      digitalWrite(15, HIGH);
      digitalWrite(16, HIGH);
      digitalWrite(17, LOW);
      break;
    //--------------------------------------
    case 4://Y4
      //001 = 1
      digitalWrite(15, LOW);
      digitalWrite(16, LOW);
      digitalWrite(17, HIGH);
      break;
    //--------------------------------------
    case 5://Y5
      //101 = 5
      digitalWrite(15, HIGH);
      digitalWrite(16, LOW);
      digitalWrite(17, HIGH);
      break;
    //--------------------------------------
    case 6://Y6
      //011 = 3
      digitalWrite(15, LOW);
      digitalWrite(16, HIGH);
      digitalWrite(17, HIGH);
      break;

    //--------------------------------------

    case 7:
      //111 = 7
      digitalWrite(15, HIGH);
      digitalWrite(16, HIGH);
      digitalWrite(17, HIGH);
      break;
    //--------------------------------------

    default:
      Serial.println("incorrect channel");
      break;
  }
  int value = analogRead(MUX_OUTPUT) * 204.6 / bitsPerVolts;
  //This for an accurate analog Read
  // bitsPerVolts = 1023.0/(readVccMilliVolts()*0.001);

  //once the measurement is made
  //set the channel to 0
  digitalWrite(15, LOW);
  digitalWrite(16, LOW);
  digitalWrite(17, LOW);

  //turn off the emitter
  digitalWrite(IR_EMIT_PIN, LOW);
  return value;
}

/*********************************************************
 * byte setMotors( int speedM1, int speedM2)
 * -------------------------------------------------------
 * This Method takes in 2 integers
 * The max and mins of the speed is -255 and +255
 * This is because they rely on the analogWrite() method which
 * has only 8 bit resolution.
 * Calling this method will create the PWM motor drive signal
 ********************************************************/
void setMotors( int speedM1, int speedM2)
{
 
  //----------------------------------------
  //if M1 speed is Positive
  if (speedM1 > 0)
  {
    analogWrite(MOTOR_LEFT_A, speedM1);
    analogWrite(MOTOR_LEFT_B, 0);
  }
  //if M1 speed is Negative
  else if (speedM1 < 0)
  {
    analogWrite(MOTOR_LEFT_A, 0);
    analogWrite(MOTOR_LEFT_B, -1 * speedM1);
  }
  //otherwise stop the left motor
  else
  {
    analogWrite(MOTOR_LEFT_A, 0);
    analogWrite(MOTOR_LEFT_B, 0);
  }
  //----------------------------------------
  //if M2 speed is Positive
  if (speedM2 > 0)
  {
    analogWrite(MOTOR_RIGHT_A, speedM2);
    analogWrite(MOTOR_RIGHT_B, 0);
  }
  //if M2 speed is Negative
  else if (speedM2 < 0)
  {
    analogWrite(MOTOR_RIGHT_A, 0);
    analogWrite(MOTOR_RIGHT_B, -1 * speedM2);
  }
  //otherwise stop the right motor
  else
  {
    analogWrite(MOTOR_RIGHT_A, 0);
    analogWrite(MOTOR_RIGHT_B, 0);
  }
  //----------------------------------------
}

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

void displayAllIRsensors()
{
  //there are normally 8 channels but only 6 are used
  // for (int i = 0; i < 8; i++)
  for (int i = 1; i < 7; i++) {
    //go through all the IR sensors
    Serial.print(readthisIRSensors(i, true));
    Serial.print(",");
    // delay(2);        // delay in between reads for stability
  }
  //Create a new line
  Serial.println("");
}


void motorTest(int delayIntervals) {
//This method test the motors using both methods
//setMotorState();
//setMotors();

  //go through all the motor states
  for (int i = 0; i < 9; i = i + 1)
  {
    //
    for (int j = 0; j < 255; j = j + 5)
    {
      setMotorState(i, j, j);
      delay(delayIntervals);
    }
    for (int j = 255; j > 0; j = j - 5)
    {
      setMotorState(i, j, j);
      delay(delayIntervals);
    }
  }
  
  //Also go through the other set motor state function
    for (int i = 0; i < 254; i++)
  {
    setMotors(i, i);
    delay(6);
  }
  for (int i = 254; i > -254; i--)
  {
    setMotors(i, i);
    delay(6);
  }
    for (int i = -254; i < 254; i++)
  {
    setMotors(i, i);
    delay(6);
  }
   for (int i = 254; i > 0; i--)
  {
    setMotors(i, i);
    delay(6);
  }
}


void turnHeading(int desiredAngle, int speedMultiplier) {

  //before doing anything make sure we are at a dead stop
  setMotorState(STOP, 0, 0);
  //this way we make a good reading on nthe MPU6050


  //speed multiplier is a integer percentage from 0 -100
  //here we convert it to 0-1 ex: 50%->0.5
  double speedMultiplierDecimal = speedMultiplier * 0.01;

  //grab initial gyro angle
  mpu6050.update();



  double initialAngleZ = mpu6050.getAngleZ() + desiredAngle;
  //if the desired angle is 0 then we will continue to drive straight ahead

  int leftMotorSpeed = 127;
  int rightMotorSpeed = 127;
  int errorMultiplier = 1;
  //we actually want the error multiplier to be proportinal to the
  //actual difference in actual angle and desired angle.

  //FUTURE may want to optimize the turn to desired angle
  //say you want 350 and you are at angle 10, instead of increasing
  //doing 10, 20,..340, 350
  //we should do 10, 0 ,350.

  while (1) {
    //grab update from the MPU6050 gyroscope
    mpu6050.update();

    //this GyroError is a value that will be produced by finding the difference
    //of the angle we want and the angle measured. We multiply by a error Multiplier
    // in order to speed of the correction
    double gyroError = errorMultiplier * (initialAngleZ - mpu6050.getAngleZ());

    //set a clamp so that the motor PWM value
    //never maxes out at 255 or minimizes to 0
    //(127+ gyroError) < 255 and (127 - gyroError) > 0
    if (gyroError > 127) {
      gyroError = 127;
    }
    else if (gyroError < -127) {
      gyroError = -127;
    }


    Serial.print(" xacc: ");
    Serial.print(mpu6050.getAccX() * 9.8);
    Serial.print(" yacc: ");
    Serial.print(mpu6050.getAccY() * 9.8);
    //each motor will be proprtional to the error
    leftMotorSpeed = (127 - gyroError) * speedMultiplierDecimal;
    rightMotorSpeed = (127 + gyroError) * speedMultiplierDecimal;
    //    Serial.print("desiredZ: ");
    //    Serial.print(desiredAngle);
    //    Serial.print("AngleZ: ");
    //    Serial.print(mpu6050.getAngleZ());
    //    Serial.print(" gyroError:");
    //    Serial.print(gyroError);
    Serial.print(" left motor: ");
    Serial.print(leftMotorSpeed);
    Serial.print(" right motor: ");
    Serial.println(rightMotorSpeed);

    if (gyroError < 0.1 && gyroError > -0.1)
    {
      break;
    }
    //set the motor values
    setMotorState(FORWARD, leftMotorSpeed, rightMotorSpeed);
  }
  setMotorState(STOP, 0, 0);
}



void turnAndFace(int desiredAngle, int speedMultiplier) {

  //before doing anything make sure we are at a dead stop
  setMotorState(STOP, 0, 0);
  //this way we make a good reading on the MPU6050

  int leftMotorSpeed = 0;
  int rightMotorSpeed = 0;
  int minSpeed = 50;
  int errorMultiplier = 4;

  //speed multiplier is a integer percentage from 0 -100
  //here we convert it to 0-1 ex: 50%->0.5
  double speedMultiplierDecimal = speedMultiplier * 0.01;

  //grab initial gyro angle
  mpu6050.update();

  double initialAngleZ = mpu6050.getAngleZ() + desiredAngle;
  //if the desired angle is 0 then we will continue to drive straight ahead

  //we actually want the error multiplier to be proportinal to the
  //actual difference in actual angle and desired angle.

  //FUTURE may want to optimize the turn to desired angle
  //say you want 350 and you are at angle 10, instead of increasing
  //doing 10, 20,..340, 350
  //we should do 10, 0 ,350.

  while (1) {
    //grab update from the MPU6050 gyroscope
    mpu6050.update();

    //this GyroError is a value that will be produced by finding the difference
    //of the angle we want and the angle measured. We multiply by a error Multiplier
    // in order to speed of the correction
    double gyroError = errorMultiplier * (initialAngleZ - mpu6050.getAngleZ());

    //set a clamp so that the motor PWM value
    //never maxes out at 255 or minimizes to 0
    //(127+ gyroError) < 255 and (127 - gyroError) > 0
    if (gyroError > 255) {
      gyroError = 255;
    }
    else if (gyroError < -255) {
      gyroError = -255;
    }

    //each motor will be proprtional to the error
    leftMotorSpeed = abs(gyroError) * speedMultiplierDecimal;
    rightMotorSpeed = abs(gyroError) * speedMultiplierDecimal;
    if (leftMotorSpeed < minSpeed && abs(gyroError) > 0) {
      leftMotorSpeed = minSpeed;
    }
    if (rightMotorSpeed < minSpeed && abs(gyroError) > 0) {
      rightMotorSpeed = minSpeed;
    }

    //check the direction
    //if I want to go to 90 and I am at 0 then diff is + and I will want to turn left
    if (gyroError < -10) {
      setMotorState(LEFT_TURN, leftMotorSpeed, rightMotorSpeed);
      //left turn is positive left motor
      //and negative on right motor
    }
    //otherwise i want to turn right
    else if (gyroError > 10) {
      setMotorState(RIGHT_TURN, leftMotorSpeed, rightMotorSpeed);
    }
    if (abs(gyroError) < 10  )
    {
      leftMotorSpeed = 0;
      rightMotorSpeed = 0;
      setMotorState(STOP, leftMotorSpeed, rightMotorSpeed);
    }

    Serial.print("desiredZ: ");
    Serial.print(desiredAngle);
    Serial.print(" AngleZ: ");
    Serial.print(mpu6050.getAngleZ());
    Serial.print(" gyroError:");
    Serial.print(gyroError);
    Serial.print(" left motor: ");
    Serial.print(leftMotorSpeed);
    Serial.print(" right motor: ");
    Serial.println(rightMotorSpeed);






  }

}


void setHeading(int desiredAngle, int speedMultiplier) {
  //speed multiplier is a integer percentage from 0 -100
  //here we convert it to 0-1 ex: 50%->0.5
  double speedMultiplierDecimal = speedMultiplier * 0.01;

  //grab initial gyro angle
  mpu6050.update();

  double initialAngleZ = mpu6050.getAngleZ() + desiredAngle;
  //if the desired angle is 0 then we will continue to drive straight ahead

  int leftMotorSpeed = 127;
  int rightMotorSpeed = 127;
  int errorMultiplier = 2;
  //we actually want the error multiplier to be proportinal to the
  //actual difference in actual angle and desired angle.

  //FUTURE may want to optimize the turn to desired angle
  //say you want 350 and you are at angle 10, instead of increasing
  //doing 10, 20,..340, 350
  //we should do 10, 0 ,350.

  for (int i = 0; i < 1000; i++) {
    //grab update from the MPU6050 gyroscope
    mpu6050.update();

    //this GyroError is a value that will be produced by finding the difference
    //of the angle we want and the angle measured. We multiply by a error Multiplier
    // in order to speed of the correction
    int gyroError = errorMultiplier * (initialAngleZ - mpu6050.getAngleZ());

    //set a clamp so that the motor PWM value
    //never maxes out at 255 or minimizes to 0
    //(127+ gyroError) < 255 and (127 - gyroError) > 0
    if (gyroError > 127) {
      gyroError = 127;
    }
    else if (gyroError < -127) {
      gyroError = -127;
    }
    //each motor will be proportional to the error
    leftMotorSpeed = (127 - gyroError) * speedMultiplierDecimal;
    rightMotorSpeed = (127 + gyroError) * speedMultiplierDecimal;
    //    Serial.print("desiredZ: ");
    //    Serial.print(desiredAngle);
    //    Serial.print("AngleZ: ");
    //    Serial.print(mpu6050.getAngleZ());
    //    Serial.print(" gyroError:");
    Serial.print(gyroError);
    Serial.print(" left motor: ");
    Serial.print(leftMotorSpeed);
    Serial.print(" right motor: ");
    Serial.println(rightMotorSpeed);

    //set the motor values
    setMotorState(FORWARD, leftMotorSpeed, rightMotorSpeed);
  }

}

void playNote(int note, int duration) {
  long initialTime = millis();

  while ( millis() - initialTime < duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(note / 2); // period divided by 2 (50% duty cycle)
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(note / 2);
  }
}


void playMelody( int melody[], int beats[], int tempo, int pause) {
  int MAX_COUNT = sizeof(melody);
  long tempos = tempo;
  int pauses = pause;
  int note;
  int beat;
  long duration;

  for (int i = 0; i < MAX_COUNT; i++) {
    note = melody[i];
    beat = beats[i];
    duration = tempos / beat; // Set up timing
    playNote(note, duration);
    delayMicroseconds(pauses);
  }
}

void excited() {
  int excited[] = {955, 758, 638, 638, 758, 638};
  int beats[] = {8, 8, 8, 4, 10, 8};
  int MAX_COUNT = sizeof(excited);
  long tempo = 1000;
  int pause = 700;
  int note;
  int beat;
  long duration;

  for (int i = 0; i < MAX_COUNT; i++) {
    note = excited[i];
    beat = beats[i];
    duration = tempo / beat; // Set up timing
    playNote(note, duration);
    delayMicroseconds(pause);
  }
}


int checkIRreceiver()
{
  int IRresult = 0;
  if (irrecv.decode(&results)) {
    IRresult = results.value;
    irrecv.resume(); // Receive the next value
    //delay(50);
  }
  return IRresult;
}
