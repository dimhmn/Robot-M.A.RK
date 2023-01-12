#include "Ultrasonic.h"
#define pinUSFront  8
#define pinUSLeft   12
#define pinUSRight  10

// Boutons face avant --> bumper
#define BPG 2
#define BPD 3
 int defaut;

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
#define K 20
//const int CorrecteurV = (Vmax-Stop)/30;
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
  //bumper
  pinMode(BPG, INPUT_PULLUP);
  pinMode(BPD, INPUT_PULLUP);
  //lcd
  //RGB bar battery
  //Ir
  //joystick
  //bluetooth
}

int bumper(int erreur){
  int buttonStateG = 0;         // Variable pour lire l'état des boutons
  int buttonStateD = 0;         

  buttonStateG = digitalRead(BPG);  // lecture des états des bumpers en face avant
  buttonStateD = digitalRead(BPD);

  if(buttonStateG ==0 || buttonStateD == 0 || erreur == 1) // Etat initial des boutons à 1 
  {                                                        // Si un des 2 boutons est à 0 --> les moteurs s'arrêtent
    StopMoteurGD;                                          // erreur = 1 permet de revenir dans la boucle et d'arrêter les moteurs définitivement
    erreur = 1;
  }
  return erreur;
}

void loop() {
  // put your main code here, to run repeatedly:
  defaut = bumper(defaut);
  Courant = analogRead(currentMoteurR);

  long TempsCourant = 0;
  Serial.println(Courant);

  if(Courant<650)
  {
    TempsCourant = millis();
  }
  else if((millis()-TempsCourant)>5000)
  {
    defaut = 1;
    //Serial.println(millis()-TempsCourant);
  }

  /*Serial.print("Temps loop : ");
  Serial.println(millis() - Temps);
  Serial.print("Courant moteur : ");
  Serial.println((Courant/5));*/
  
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
    //tourne a gauche
    float x = 1-float(USDistRight)/float(USDistLeft);
    //int RM = (-(2*pow(x,2)))*(x-1.5)*K;
    int RM = 0;
    if (x<0.4) {
      RM = 4;
    } else{
      RM = 15;
    }
    Serial.print("RM : ");
    Serial.println(x);
    VMoteurG = Vmax - RM;
    VMoteurD = Vmax;
  } else if (USDistLeft < USDistRight) {
    //tourne a droite
    float x = 1-float(USDistLeft)/float(USDistRight);
    //int RM = int((-(2*pow(x,2)))*(x-1.5)*K);
    int RM = 0;
    if (x<0.4) {
      RM = 4;
    } else{
      RM = 15;
    }
    VMoteurG = Vmax;
    VMoteurD = Vmax - RM;
  } else{
    //tout droit
    VMoteurG = Vmax;
    VMoteurD = Vmax;
  }
  
  MoteurGD(VMoteurG,VMoteurD);
  
}
