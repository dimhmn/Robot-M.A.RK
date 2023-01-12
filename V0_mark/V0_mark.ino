#include "Ultrasonic.h"
#define pinUSFront  8
#define pinUSLeft   12
#define pinUSRight  10

//define US
Ultrasonic USFront(pinUSFront);
Ultrasonic USRight(pinUSRight);
Ultrasonic USLeft(pinUSLeft);
long USDistFront;
long USDistLeft;
long USDistRight;

int currentMoteurR = A5;
int Courant = 0;


//define moteur

#define Thash 800
#define Stop 400
#define Vmax 600
const int CorrecteurV = (Vmax-Stop)/30;
long VMoteurG;
long VMoteurD;


// Macros
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)


void initMoteurs() {  // MoteurG :OC5A=PIN46-PL3, MoteurD : OC5B=PIN45-PL4
  DDRL = 0x18 ; // PL3 et PL4
  DDRB = 0x80 ; // PB7 LedToggle
  // COM5B_1:0 = 10   -> clear sur egalite++, set sur egalite--
  // WGM5_3:1 = 1000  -> mode 8 => ICR5 defini le TOP
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50); // CS_12:10  = 001  -> prediv par 1
  ICR5 = Thash; // 1999 correspond a f = 4khz
  StopMoteurGD;
  // Interrution de débordement du timer
  TIMSK5 = 1 << TOIE5;
}

ISR (TIMER5_OVF_vect) { // Pour la lecture du courant
  LedToggle;
}

void setup() {
  Serial.begin(115200);
  //ultrason
  //servo
  //moteurs
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);

  MoteurGD(400, 400);
  //impact
  //lcd
  //RGB bar battery
  //Ir
  //joystick
  //bluetooth
}

void loop() {
  // put your main code here, to run repeatedly:

  Courant = analogRead(currentMoteurR);

  long Temps = millis();
  
  USDistFront = USFront.MeasureInCentimeters();
  Serial.print("Front = ");
  Serial.println(USDistFront);
  USDistRight = USRight.MeasureInCentimeters();
  Serial.print("Right = ");
  Serial.println(USDistRight);
  USDistLeft = USLeft.MeasureInCentimeters();
  Serial.print("Left = ");
  Serial.println(USDistLeft);
  Serial.println();

  if (USDistLeft > USDistRight) {
    int reductionMoteur = int(float(CorrecteurV)*(1-(float(USDistRight)/float(USDistLeft))));
    //int reductionMoteur = int(float(CorrecteurV)*(float(USDistLeft)/float(USDistRight)));
    Serial.print("RD = ");
    Serial.println(reductionMoteur);
    VMoteurG = Vmax - reductionMoteur;
    VMoteurD = Vmax;
  } else if (USDistLeft < USDistRight) {
   int reductionMoteur = int(float(CorrecteurV)*(1-(float(USDistLeft)/float(USDistRight))));
   //int reductionMoteur = int(float(CorrecteurV)*(float(USDistRight)/float(USDistLeft)));
    Serial.print("RD = ");
    Serial.println(reductionMoteur);
    VMoteurG = Vmax;
    VMoteurD = Vmax - reductionMoteur;
  } else{
    VMoteurG = Vmax;
    VMoteurD = Vmax;    
  }
  
  MoteurGD(VMoteurG,VMoteurD);
  Serial.print("Temps loop : ");
  Serial.println(millis() - Temps);
  Serial.print("Courant moteur : ");
  Serial.println((Courant/5));
}
