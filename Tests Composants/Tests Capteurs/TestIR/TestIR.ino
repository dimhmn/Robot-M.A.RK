#define IR 6
#include "rgb_lcd.h"

rgb_lcd lcd;


// variables will change:
int IRState = 0;         // variable for reading the pushbutton status


void setup() {
  lcd.begin(16, 2);
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(IR, INPUT_PULLUP);
}

void loop() {
  // read the state of the pushbutton value:
  IRState = digitalRead(IR); 
  Serial.println(IRState);
  lcd.clear();
  lcd.print(IRState);
  delay(100);
}
