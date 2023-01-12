#include <Bridge.h>
#include <BridgeClient.h>
#include <BridgeServer.h>
#include <BridgeSSLClient.h>
#include <BridgeUdp.h>
#include <Console.h>
#include <FileIO.h>
#include <HttpClient.h>
#include <Mailbox.h>
#include <Process.h>
#include <YunClient.h>
#include <YunServer.h>

#include "Ultrasonic.h"
#include <Encoder.h>
#include "rgb_lcd.h"

#define pinUSFront        8
#define pinUSLeft         12
#define pinUSRight        10
#define pinIR             6
#define pinBpG            2
#define pinBpD            3
#define pinCurrentLeft   A5
#define pinCurrentRight   A4

//encodeurs
Encoder knobLeft(18, 29);
Encoder knobRight(19, 27);
long positionLeft  = -999;
long positionRight = -999;
long tourRoueDroite;
long tourRoueGauche;
 
//define US
Ultrasonic USFront(pinUSFront);
Ultrasonic USRight(pinUSRight);
Ultrasonic USLeft(pinUSLeft);

long USDistFront = USFront.MeasureInCentimeters();
long USDistRight = USRight.MeasureInCentimeters();
long USDistLeft = USLeft.MeasureInCentimeters();

//lcd
rgb_lcd lcd;

//mesure courant moteur
int currentMoteurR = A5;
int Courant = 0;

// arret urgence
bool AU = false;


//define moteur
#define Thash 800
#define Stop 400
#define Vmax 600
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

//ISR (TIMER5_OVF_vect) { // Pour la lecture du courant
//  //LedToggle;
//}

//////////////////////////////////////////
// Initialisation du timer4 pour bumper 
void init_bumper() {
  TCCR4B =(1 << CS42) + (1<< CS40); // prediv 1024
  TIMSK4 = 1 << TOIE4;
}
// Interruption pour la lecture des bumpers
ISR (TIMER4_OVF_vect) { 
  LedToggle;
  bool BPG = !digitalRead(pinBpG);
  bool BPD = !digitalRead(pinBpD);
  if (BPG || BPD) AU = true;
  int US = analogRead(pinUSFront);
  
  long USDistFront = USFront.MeasureInCentimeters();
  long USDistRight = USRight.MeasureInCentimeters();
  long USDistLeft = USLeft.MeasureInCentimeters();
  if (USDistFront < 10 || USDistRight < 10 || USDistLeft < 10) AU = true;
  LedToggle;        
}
///////////////////////////////////////////
// Timer3 pour ultrasons
/*
void ultrasons() {
  TCCR3B =(1 << CS32)+(1 << CS30); // prediv 8
  TIMSK3 = 1 << TOIE3;
}
// Interruption pour la lecture des ultrasons
ISR (TIMER3_OVF_vect) { 
  if (USDistFront < 10 || USDistRight < 10 || USDistLeft < 10) AU = true;
}*/
///////////////////////////////////////////




void setup() {
  Serial.begin(9600);

  //moteurs
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();
  init_bumper();
  //ultrasons();
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

  //encodeurs
  knobLeft.write(0);
    knobRight.write(0);
}

void loop() {
  
  
  
  
  
  
  
  
  
  
  
  
  
// //acquisition capteur infra rouge
//  int IRState = digitalRead(pinIR);
//
//  //test blocage roue
// /* static long t;
//  if (VMoteurG == 400) {
//    t = millis();
//  }
//  if (t + 100 < millis()) {
//    t = millis();
//    long newRight = knobRight.read();
//    long newLeft = knobLeft.read();
//    if (VMoteurG != 400 && positionLeft > newLeft - 5 && positionLeft < newLeft + 5) AU = true;
//    if (VMoteurD != 400 && positionRight > newRight - 5 && positionRight < newRight + 5) AU = true;
//    if (positionRight != newRight || positionLeft != newLeft) {
//      positionRight = newRight;
//      positionLeft = newLeft;
//    }
//  }*/
//
//  if (!AU) {
//    //test Infra rouge
//    if (IRState) {
//      VMoteurG = 400;
//      VMoteurD = 400;
//    } else {
//      // acquisition données capteurs ultrason
//      /*long USDistFront = USFront.MeasureInCentimeters();
//      long USDistRight = USRight.MeasureInCentimeters();
//      long USDistLeft = USLeft.MeasureInCentimeters();
//      if (USDistFront < 10 || USDistRight < 10 || USDistLeft < 10) AU = true;
//      */
//      
//      //test courant max
//      int CourantR = analogRead(pinCurrentRight);
//      int CourantG = analogRead(pinCurrentLeft);
//      Courant = (CourantG < CourantR) ? CourantR : CourantG;
//      long TempsCourant = 0;
//
//      if (Courant < 650)      {
//        TempsCourant = millis();
//      } else if ((millis() - TempsCourant) > 2000)      {
//        AU = true;
//      }
//
//      //deplacement
//      if (USDistLeft > USDistRight) {
//        //tourne a gauche
//        VMoteurG = Vmax - 10;
//        VMoteurD = Vmax;
//      } else if (USDistLeft < USDistRight) {
//        //tourne a droite
//        VMoteurG = Vmax;
//        VMoteurD = Vmax - 25;
//      } else {
//        //tout droit
//        VMoteurG = Vmax;
//        VMoteurD = Vmax;
//      }
//    }
//  } else { //si arret urgence
//    VMoteurG = 400;
//    VMoteurD = 400;
//  }
//  long newLeft, newRight;
//  newLeft = knobLeft.read();
//  newRight = knobRight.read();
//  if (newLeft != positionLeft || newRight != positionRight) {
//    positionLeft = newLeft;
//    positionRight = newRight;
//  }
//  //affichage de l'arret d'urgence
//  static bool b = false;
//  if (AU && !b) {
//    b = !b;
//    //lcd.clear();
//   // lcd.write("AU");
//    lcd.clear();
//    
//    lcd.print((newLeft/1200));
//    lcd.print(" ");
//    
//    lcd.print((newRight/1200));
//  }
//
//
//  MoteurGD(VMoteurG, VMoteurD);
 
}
