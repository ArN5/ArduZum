#include<IRremote.h>
#include <Wire.h>
#include "U8glib.h"

//OLED class
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);


int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;

int currentState = 1; 
int totalStates = 5;

void setup() {

  //IR SETUP
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  Serial.println(currentState);

  u8g.firstPage();
  u8g.setFont(u8g_font_10x20);
  //u8g.setFont(u8g_font_unifont);
  
  
  
}

//*********************************************************************

void loop() {

  u8g.firstPage();
  do {
  u8g.setPrintPos(60,20);
  u8g.print(currentState);
  }
  while(u8g.nextPage());
  //delay(1000);

  if (irrecv.decode(&results)) // if Zumi detects a button from remote:
  {
    
    // ***********************************************
    if (results.value == 0xE0E020DF) // Left
    {
      if (currentState == 1) {
        currentState = totalStates;
      }

      else {
        currentState -= 1;
      }

    }
    // ***********************************************
    else if (results.value == 0xE0E010EF) // Right
    {
      if (currentState == totalStates) {
        currentState = 1;
      }

      else {
        currentState += 1;
      }
    }
    // ***********************************************
    Serial.println(currentState);
    irrecv.resume();
  }

  delay(600);

  
}



