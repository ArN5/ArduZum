#include "notes.h"
#define BUZZER_PIN 14

void setup() {
  pinMode(BUZZER_PIN, OUTPUT);
}

void loop() {

  //excited();
  

  playNote(C4, 100);
  delay(300);
  playNote(D4, 100);
  delay(300);
  playNote(E4, 100);
  delay(300);
  playNote(F4, 100);
  delay(300);
  playNote(G4, 100);
  delay(300);
  playNote(A4, 100);
  delay(300);
  playNote(B4, 100);
  delay(300);
  playNote(C5, 100);
  delay(300);
  

  delay(1000);
}


//************************************************************************
void playNote(int note, int duration) {

  long initialTime = millis();

  while ( millis() - initialTime < duration) {

    digitalWrite(BUZZER_PIN, HIGH);
    delayMicroseconds(note / 2); // period divided by 2 (50% duty cycle)
    digitalWrite(BUZZER_PIN, LOW);
    delayMicroseconds(note / 2);

  }

}

void excited() {

  int excited[] = {C6, E6, G6, G6, E6, G6};
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

//************************************************************************
