#include<IRremote.h>
#include <Wire.h>


int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;

int currentState = 1; 

void setup() {

  //IR SETUP
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  Serial.println(currentState);
}

//*********************************************************************

void loop() {

  if (irrecv.decode(&results)) // if Zumi detects a button from remote:
  {
    
    // ***********************************************
    if (results.value == 0xE0E020DF) // Left
    {
      if (currentState == 1) {
        currentState = 5;
      }

      else {
        currentState -= 1;
      }

    }
    // ***********************************************
    else if (results.value == 0xE0E010EF) // Right
    {
      if (currentState == 5) {
        currentState = 1;
      }

      else {
        currentState += 1;
      }
    }
    // ***********************************************
    Serial.println(currentState);

  }


  irrecv.resume();
  delay(600);

}


