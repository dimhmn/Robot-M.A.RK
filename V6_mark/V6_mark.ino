#include "Ultrasonic.h"
#include <Encoder.h>
#include "rgb_lcd.h"
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

#define pinUSFront        8
#define pinUSLeft         12
#define pinUSRight        10
#define pinIR             6
#define pinBpG            2
#define pinBpD            3
#define pinKnobLeft      18,29
#define pinKnobRight     19,27
#define pinCurrentLeft    A5
#define pinCurrentRight   A4
#define pinJoystickX      A3
#define pinJoystickY      A2

#define DELTAKNOB 5
#define TEMPS_PARCOURS_MAX 5 //secondes
#define USDDISTMIN 15        //centimètres


//define moteur
#define Thash 800
#define Stop 400
#define Vmax 600
#define K 20


// Macros Moteur / ne pas modifier
#define LedToggle digitalWrite(13, !digitalRead(13))
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

//asservisement
float Kp = 1;
bool uDisplay = false;

//Gestion Etat du Robot
enum Etat {
  INIT,             //Initialisation du robot a la mise sous tension
  ATTENTE,          //Attente du demarrage du robot par une entree utilisateur
  DEMARRAGE_ATTENTE,//Attente que le Robot soit posé au sol pendant 3 secondes
  DEMARRAGE,        //Demarrage du robot apres l'entree utilisateur
  MARCHE,           //Execution du code de déplacement
  AU,               //Arret d'Urgence lors du fonctionnement
  TEMPSDEPASSE,     //Après 10 minutes de fonctionnement
  ARRIVE            //Lors de la détection de la ligne noire
};

enum CodeAU {
  DEFAUT,
  BLOCAGE,      //blocage des roues
  DISTANCE,     //distance trop faible / bumpers
  COURANT,      //courant trop important
  ROTATION      //rotation trop rapide
};


//gestion joystick
enum JoystickState {
  REPOS,
  DROITE,
  GAUCHE,
  HAUT,
  BAS,
  CLIC
};


//declaration des enums
volatile enum Etat etat = INIT;
volatile enum CodeAU codeAU = DEFAUT;



//capteurs ultrason
volatile long USDistFront = 0, USDistLeft = 0, USDistRight = 0;

//Gestion Temps de Parcours
volatile unsigned long tempsDemarrage = 0;
volatile unsigned long tempsArret = 0;
volatile bool tempsDepasse = false;


//encodeurs
Encoder knobLeft(pinKnobLeft);
Encoder knobRight(pinKnobRight);
volatile long oldKnobLeft  = -999;
volatile long oldKnobRight = -999;
volatile long newKnobRight = 0;
volatile long newKnobLeft = 0;
volatile long NbTourDeRoues;
volatile float distanceParcouru;

//define US
Ultrasonic USFront(pinUSFront);
Ultrasonic USRight(pinUSRight);
Ultrasonic USLeft(pinUSLeft);

//lcd
rgb_lcd lcd;

//mesure courant moteur
volatile int currentMoteurR = A5;
volatile int Courant = 0;

// gyroscope
LSM6DS3 gyro( I2C_MODE, 0x6A );  //I2C device address 0x6A
volatile float vitesseRotation = 0;

//Capteur infrarouge
volatile bool IRState = 0;

//Capteur Bumpers
volatile bool BPD = false;
volatile bool BPG = false;

//vitesse roues moteur
volatile long VMoteurG;
volatile long VMoteurD;



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


ISR (TIMER5_OVF_vect) {
  LedToggle;
  static int compteurUS_T5 = 0;
  static int compteurIR_T5 = 0;
  static int compteurKnob_T5 = 0;
  static int compteurGyro_T5 = 0;
  static int compteurBumper_T5 = 0;
  if (etat == MARCHE) {
    compteurUS_T5++;
    compteurIR_T5++;
    compteurKnob_T5++;
    compteurGyro_T5++;
    compteurBumper_T5++;

    // Interruption pour les ultrasons
    if (compteurUS_T5 > 750) {
      USDistFront = USFront.MeasureInCentimeters();
      USDistRight = USRight.MeasureInCentimeters();
      USDistLeft = USLeft.MeasureInCentimeters();
      compteurUS_T5 = 0;
    }

    // Interruption pour l'infrarouge
    if (compteurIR_T5 > 250) {
      IRState = digitalRead(pinIR);
      compteurIR_T5 = 0;
    }

    if (compteurKnob_T5 == 200) {
      oldKnobRight = newKnobRight;
      oldKnobLeft = newKnobLeft;
      newKnobRight = knobRight.read();
      newKnobLeft = knobLeft.read();
      compteurKnob_T5 = 0;
    }
    if (compteurGyro_T5 == 100) {
      compteurGyro_T5 = 0;
      vitesseRotation = gyro.readFloatGyroZ();
    }

    if (compteurBumper_T5 == 150) {
      BPG = digitalRead(pinBpG);
      BPD = digitalRead(pinBpD);
    }
  }
  LedToggle;
}




void afficher( char* ligne1, char* ligne2) {
  lcd.clear();
  lcd.setRGB(255, 255, 255);
  lcd.setCursor(0, 0);
  lcd.write(ligne1);
  lcd.setCursor(0, 1);
  lcd.write(ligne2);
}

//gestion joystick
int getJoystickState() {
  int x = analogRead(pinJoystickX);
  int y = analogRead(pinJoystickY);
  JoystickState etatJoystick;
  etatJoystick = REPOS;
  if (x > 700) etatJoystick = GAUCHE;
  if (x < 300) etatJoystick = DROITE;
  if (y > 700) etatJoystick = HAUT;
  if (y < 300) etatJoystick = BAS;
  if (y > 1000) etatJoystick = CLIC;
  return etatJoystick;
}

void updateControls() {
  static JoystickState jsPrec;
  JoystickState js = getJoystickState();
  if (jsPrec != js) {
    jsPrec = js;
    Serial.println("tata");
    switch (etat) {
      case ATTENTE : {
        Serial.println("titi");
          if (js == CLIC) {
            Serial.println("toto");
            etat = DEMARRAGE_ATTENTE;
          } else if (js == HAUT) {
            uDisplay = true;
            Kp += 0.1;
          } else if (js == BAS && Kp >= 1) {
            uDisplay = true;
            Kp -= 0.1;
          }
          break;
        }

      case AU : {
          etat = ATTENTE;
          break;
        }
    }
  }
}


void updateDisplay() {
  static Etat etatPrec = INIT;
  Etat e = etat;
  if (etatPrec != e || uDisplay == true) {
    uDisplay = false;
    etatPrec = etat;
    switch (etat) {
      case INIT :
        afficher("INITIALISATION", "");
        break;

      case ATTENTE:

        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("   EN ATTENTE   ");
        lcd.setCursor(0, 1);
        lcd.write("Kp :");
        lcd.setCursor(5, 1);
        lcd.print(Kp, 1);
        break;
      case DEMARRAGE_ATTENTE :
        afficher(" POSER LE ROBOT ", "  ATTENDEZ 3S");
        break;
      case DEMARRAGE:
        afficher("   DEMARRAGE    ", "    EN COURS    ");
        break;

      case MARCHE:
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("temps :");
        lcd.setCursor(0, 1);
        lcd.write(int((millis() - tempsDemarrage) / 1000));
        break;

      case TEMPSDEPASSE:
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("TEMPS DEPASSE");
        lcd.setCursor(0, 1);
        lcd.write(int(tempsArret - tempsDemarrage) / 1000);
        break;

      case ARRIVE:
        lcd.clear();
        lcd.setRGB(0, 255, 0);
        lcd.print("Bravo M.A.R.K");
        break;
      case AU :
        lcd.clear();
        lcd.setRGB(255, 0, 0);
        switch (codeAU) {
          case DISTANCE: {
              lcd.print("Distance");
              break;
            }
          case BLOCAGE: {
              lcd.print("Roues Bloquees");
              break;
            }
          case COURANT: {
              lcd.print("Courant Excessif");
              break;
            }
          case ROTATION : {
              lcd.print("Rotation Excessive");
              break;
            }
        }
        break;
    }
  }
}



int main() {
  Serial.begin(9600);

  //moteurs
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();

  EICRB |= (1 << ISC40) + (1 << ISC50); // Interruptions autorisées sur tout les fronts
  EIMSK |= (1 << INT4) + (1 << INT5); // Activation des sorties d'interruption externe (4 et 5)

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

  etat = ATTENTE;
  while (1) {
    switch (etat) {
      case INIT : {
          break;
        }
      case ATTENTE : {
          break;
        }
      case DEMARRAGE_ATTENTE : {
          int IRState = digitalRead(pinIR);
          static long t = millis();
          if (IRState) {
            t = millis();
          } else {
            if (t + 1000 < millis()) {
              etat = DEMARRAGE;
            }
          }
          break;
        }
      case DEMARRAGE : {
          tempsDemarrage = millis();
          tempsArret = tempsDemarrage + TEMPS_PARCOURS_MAX * 1000;
          tempsDepasse = false;
          etat = MARCHE;
          break;
        }
      case MARCHE: {
          //acquisition capteur infra rouge et d'arrivée
          if (IRState) {
            tempsArret = millis();
            etat = ARRIVE;
          }

          //verification roue bloqué
          //si la consigne du moteur n'est pas d'etre a l'arret, alors vérifier si la roue est toujours dans sa position précédente
          if (VMoteurG != Stop && oldKnobLeft > newKnobLeft - DELTAKNOB && oldKnobLeft  < newKnobLeft + DELTAKNOB) {
            etat = AU;
            codeAU = BLOCAGE;
          }
          if (VMoteurD != Stop && oldKnobRight  > newKnobRight - DELTAKNOB && oldKnobRight  < newKnobRight + DELTAKNOB) {
            etat = AU;
            codeAU = BLOCAGE;
          }

          //si la roue a changé de position alors retenir sa nouvelle position
          if (oldKnobRight != newKnobRight || oldKnobLeft != newKnobLeft) {
            oldKnobRight = newKnobRight;
            oldKnobLeft = newKnobLeft;
            NbTourDeRoues = ((newKnobRight+newKnobLeft)/2400);
            DistanceParcouru = NbTourDeRoues*0.188496;
          }


          //capteurs ultrason
          //declencher un arret d'urgence si les capteurs sont trop près du mur
          if (USDistFront < USDDISTMIN || USDistRight < USDDISTMIN || USDistLeft < USDDISTMIN) {
            etat = AU;
            codeAU = DISTANCE;
          }

          //test courant max
          int CourantR = analogRead(pinCurrentRight);
          int CourantG = analogRead(pinCurrentLeft);
          Courant = (CourantG < CourantR) ? CourantR : CourantG;
          static unsigned long TempsCourant = millis();

          if (Courant < 650) {
            TempsCourant = millis();
          } else if (TempsCourant + 2000 < millis() ) {
            etat = AU;
            codeAU = COURANT;
          }

          //test gyroscope
          if (vitesseRotation > 200 || vitesseRotation < -200) {
            etat = AU;
            codeAU = ROTATION;
          }

          //deplacement
          if (USDistLeft > USDistRight) {
            //tourne a gauche
            float erreur = (USDistLeft - USDistRight) / 2;
            int V = 0;

            VMoteurG = Vmax - erreur;
            VMoteurD = Vmax;
          } else if (USDistLeft < USDistRight) {
            //tourne a droite
            float erreur = (USDistRight - USDistLeft) / 2;
            int V = 0;

            VMoteurG = Vmax;
            VMoteurD = Vmax - erreur;
          } else {
            //tout droit
            VMoteurG = Vmax;
            VMoteurD = Vmax;
          }
          break;
        }
      case AU: {
          //affichage de l'arret d'urgence
          VMoteurG = 400;
          VMoteurD = 400;
          break;
        }
      case TEMPSDEPASSE: {
          break;
        }
      case ARRIVE: {
          VMoteurG = 400;
          VMoteurD = 400;
          break;
        }
    }
    MoteurGD(VMoteurG, VMoteurD);
    updateDisplay();
    updateControls();
    
  }
}
