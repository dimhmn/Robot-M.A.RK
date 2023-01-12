#include "Ultrasonic.h"
#include <Encoder.h>
#include "rgb_lcd.h"
#include "SparkFunLSM6DS3.h"

#define pinUSFront        8
#define pinUSLeft         12
#define pinUSRight        10
#define pinIR             6
#define pinBpG            2
#define pinBpD            3
#define pinCurrentLeft   A5
#define pinCurrentRight   A4

//accéleromètre
LSM6DS3 gyro( I2C_MODE, 0x6A );
bool IR;
int couleur;
int tempo;

//encodeurs
Encoder knobLeft(18, 29);
Encoder knobRight(19, 27);
long positionLeft  = -999;
long positionRight = -999;

//define US
Ultrasonic USFront(pinUSFront);
Ultrasonic USRight(pinUSRight);
Ultrasonic USLeft(pinUSLeft);

//lcd
rgb_lcd lcd;

//mesure courant moteur
int currentMoteurR = A5;
int Courant = 0;

// arret urgence
int AU = 0;


//define moteur
#define Thash 800
#define Stop 400
#define VmaxD 800
#define VmaxG 800
#define K 20
long VMoteurG;
long VMoteurD;


// Macros Moteur
// /!\ ne pas changer /!\
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
  //LedToggle;
}


void setup() {
  Serial.begin(9600);

  //moteurs
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  sei();
  digitalWrite(43, 1);
  VMoteurG = 400;
  VMoteurD = 400;
  MoteurGD(VMoteurG, VMoteurD);

  //impact
  pinMode(pinBpG, INPUT_PULLUP);
  pinMode(pinBpD, INPUT_PULLUP);

  //lcd
  lcd.begin(16, 2);

  //Ir
  pinMode(pinIR, INPUT_PULLUP);
}

void loop() {
  //acquisition capteur infra rouge
  int IRState = digitalRead(pinIR);

  //test bumper
  bool BPG = !digitalRead(pinBpG);
  bool BPD = !digitalRead(pinBpD);
  if (BPG || BPD) AU = 1;

  //test blocage roue
  static long t;
  if (VMoteurG == 400) {
    t = millis();
  }
  if (t + 100 < millis()) {
    t = millis();
    long newRight = knobRight.read();
    long newLeft = knobLeft.read();
    if (VMoteurG != 400 && positionLeft > newLeft - 5 && positionLeft < newLeft + 5) AU = 2;
    if (VMoteurD != 400 && positionRight > newRight - 5 && positionRight < newRight + 5) AU = 2;
    if (positionRight != newRight || positionLeft != newLeft) {
      positionRight = newRight;
      positionLeft = newLeft;
    }
  }

  if (!AU) {
    //test Infra rouge
    if (IRState) {
      VMoteurG = 400;
      VMoteurD = 400;
    } else {
      // acquisition données capteurs ultrason
      long USDistFront = USFront.MeasureInCentimeters();
      long USDistRight = USRight.MeasureInCentimeters();
      long USDistLeft = USLeft.MeasureInCentimeters();
      if (USDistFront < 10 || USDistRight < 10 || USDistLeft < 10) AU = 1;

      //test courant max
      int CourantR = analogRead(pinCurrentRight);
      int CourantG = analogRead(pinCurrentLeft);
      Courant = (CourantG < CourantR) ? CourantR : CourantG;
      long TempsCourant = 0;

      if (Courant < 650)      {
        TempsCourant = millis();
      } else if ((millis() - TempsCourant) > 4000)      {
        AU = 3;
      }

      //test rotation 200deg/s
      if((gyro.readFloatGyroZ())>5){
        AU = 4;
      }
      
      //deplacement
      if (USDistLeft > USDistRight) {
        //tourne a gauche
        VMoteurG = VmaxG;
        VMoteurD = VmaxD+10;
      } else if (USDistLeft < USDistRight) {
        //tourne a droite
        VMoteurG = VmaxG+10;
        VMoteurD = VmaxD;
      } /*else {
        //tout droit
        VMoteurG = VmaxG;
        VMoteurD = VmaxD;
      }*/
    }
  } else { //si arret urgence
    VMoteurG = 400;
    VMoteurD = 400;
  }

  //affichage de l'arret d'urgence
  static bool b = false;
  if (AU && !b && IRState && !IR) {
    b = !b;
    lcd.clear();
    lcd.setRGB(255, 0, 0);
    switch (AU) {
       case 1: lcd.print("Distance");break;
       case 2: lcd.print("Roues Bloquees");break;
       case 3: lcd.print("Courant Excessif");break;
       case 4: lcd.print("Rotation Excessive");break;
    }
  }
   if(IRState && !IR && !AU){
    IR = true;
    lcd.clear();
    lcd.setRGB(0, 255, 0);
    lcd.print("Bravo M.A.R.K");
    lcd.setRGB(255, 0, 0);
   }
   if(IR){
    switch (couleur){
      case 0: lcd.setRGB(255,0,0);
              tempo++;
              if(tempo>5){
                tempo = 0;
                couleur = 1;
              }
              break;
       case 1: lcd.setRGB(0,255,0);
              tempo++;
              if(tempo>5){
                tempo = 0;
                couleur = 2;
              }
              break;
       case 2: lcd.setRGB(0,0,255);
              tempo++;
              if(tempo>5){
                tempo = 0;
                couleur = 0;
              }
              break;
    }
    VMoteurG = 400;
    VMoteurD = 400;
   }
  


  MoteurGD(VMoteurG, VMoteurD);
}
