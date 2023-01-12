#include "Ultrasonic.h"


#define pinUSFront        8
#define pinUSLeft         12
#define pinUSRight        10
#define LedToggle         digitalWrite(13, !digitalRead(13))
long duration;
long temps;
long centimetre;

// Initialisation du timer4 pour ultrasons 
void init_US() {
  DDRB = 0x80 ; // PB7 LedToggle
  TCCR4B =(1 << CS42) + (1<< CS40); // prediv 1024
  TIMSK4 = 1 << TOIE4;
}
// Interruption pour la lecture des bumpers
ISR (TIMER4_OVF_vect) { 
 
  //long USDistFront = USFront.MeasureInCentimeters();
  //long USDistRight = USRight.MeasureInCentimeters();
  //long USDistLeft = USLeft.MeasureInCentimeters();
  pinMode(pinUSFront, OUTPUT);
  digitalWrite(pinUSFront, LOW);
  delayMicroseconds(2);
  digitalWrite(pinUSFront, HIGH);
  delayMicroseconds(5);
  digitalWrite(pinUSFront,LOW);
  pinMode(pinUSFront,INPUT);
  duration = millis();       
}

ISR(PCINT1_vect){
  LedToggle;
  temps = millis()-duration;
}



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  init_US();
  PCICR |= (1<<PCIE1);
  PCMSK1 |= (1<<PCINT8);
  SREG |= (1<<INTF7);
  sei();
}

void loop() {
  // put your main code here, to run repeatedly:
  centimetre = temps/29/2;
  Serial.println(temps);
}
