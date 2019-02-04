
//Include libraries for IR remote, i2c, and OLED
#include<IRremote.h>
#include <Wire.h>
#include "U8glib.h"

//identify correct OLED class
//U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE|U8G_I2C_OPT_DEV_0);

int RECV_PIN = 4;
IRrecv irrecv(RECV_PIN);
decode_results results;


//*********************************************************************
void setup() {
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  u8g.firstPage();

  u8g.setFont(u8g_font_04b_24);
  //u8g.setFont(u8g_font_unifont);

}
//*********************************************************************


void loop() {

  u8g.firstPage();
  do {
    readIR();
  }
  while ( u8g.nextPage() );

  delay(100);
}


//*********************************************************************
void readIR() {
  
  String dir = "";

  if (irrecv.decode(&results))
  {

    if (results.value == 0xE0E0807F)
    {
      Serial.println("FORWARD");
      dir = "FORWARD";
    }

    else if (results.value == 0xE0E040BF)
    {
      Serial.println("BACKWARD");
      dir = "BACKWARD";
    }

    else if (results.value == 0xE0E020DF)
    {
      Serial.println("LEFT");
      dir = "LEFT";
    }

    else if (results.value == 0xE0E010EF)
    {
      Serial.println("RIGHT");
      dir = "RIGHT";

    }

    else {

    }
    
    u8g.setPrintPos(0, 7);
    u8g.print(dir);
    //u8g.setFontPosTop();
    irrecv.resume();
    
  }
}

//*********************************************************************



