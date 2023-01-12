#include "Ultrasonic.h"
#include <Encoder.h>
#include <Grove_LED_Bar.h>
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

#define pinTimer4         7
#define pinTimer5         13


#define DELTAKNOB 0
#define TEMPS_PARCOURS_MAX 5 //secondes
#define USDDISTMIN 15        //centimètres
#define CONSOHORSHACHEUR 0.55 //550mA de conso sans les moteurs
#define USDISTVINGT 20


//define moteur
#define Thash 800
#define Stop 400
#define Vmax 600
#define K 20

#define LedToggleTimer4 digitalWrite(pinTimer4, !digitalRead(pinTimer4))
#define LedToggleTimer5 digitalWrite(pinTimer5, !digitalRead(pinTimer5))

// Macros Moteur / ne pas modifier
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

String nomCapteur;

enum MenuARRIVE {
  BRAVO,
  CONSO,
  TPSPARCOURS,
  VINGTCM,
  TRROUES,
  DISTANCEPARCOURUS
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
volatile enum MenuARRIVE menuARRIVE = BRAVO;

//gestion des 3s d'attente avant le démarrage
volatile bool resetTimerDemarrage = false;
volatile unsigned long timeStart = millis();

//capteurs ultrason
volatile long USDistFront = 1000, USDistLeft = 1000, USDistRight = 1000;
volatile int Nbvingt; // Nb de détections < 20cm
volatile bool FM_Vingt;

//Gestion Temps de Parcours
volatile unsigned long tempsDemarrage = 0;
volatile unsigned long tempsArret = 0;
volatile bool tempsDepasse = false;
volatile unsigned long compteurTimerFin = 0;


//encodeurs
Encoder knobLeft(pinKnobLeft);
Encoder knobRight(pinKnobRight);
volatile long oldKnobLeft  = -999;
volatile long oldKnobRight = -999;
volatile long newKnobRight = 0;
volatile long newKnobLeft = 0;

//define US
Ultrasonic USFront(pinUSFront);
Ultrasonic USRight(pinUSRight);
Ultrasonic USLeft(pinUSLeft);

//lcd
rgb_lcd lcd;

// Batterie
Grove_LED_Bar bar(5, 4, 0);  // Clock pin, Data pin, Orientation
volatile long  cptTension=0;
volatile int affichageTension;
volatile long led;
    

//mesure courant moteur
volatile int Courant = 0;
volatile int CourantR;
volatile int CourantG;
volatile float Energie;
volatile int nombre;
volatile float moyenne;

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

volatile bool _t = false;

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

ISR(TIMER5_OVF_vect) {
  LedToggleTimer5;
}

ISR (TIMER4_OVF_vect) {
  LedToggleTimer4;
  TCNT4 = 40535;

  static int compteurUS_T5 = 0;
  static int compteurKnob_T5 = 1;
  static int compteurGyro_T5 = 2;
  if (etat == MARCHE || etat == DEMARRAGE_ATTENTE || etat == DEMARRAGE) {
    compteurUS_T5++;
    compteurKnob_T5++;
    compteurGyro_T5++;

    // Interruption pour les ultrasons
    if (compteurUS_T5 >= 3) {
      USDistFront = USFront.MeasureInCentimeters();
      USDistRight = USRight.MeasureInCentimeters();
      USDistLeft = USLeft.MeasureInCentimeters();
      compteurUS_T5 = 0;
    }

    IRState = digitalRead(pinIR);

    //encodeurs pour le blocage des roues
    if (compteurKnob_T5 >= 3) {
      oldKnobRight = newKnobRight;
      oldKnobLeft = newKnobLeft;
      newKnobRight = knobRight.read();
      newKnobLeft = knobLeft.read();
      compteurKnob_T5 = 0;
    }

    //bumpers
    BPG = digitalRead(pinBpG);
    BPD = digitalRead(pinBpD);
  }


  if (etat == MARCHE) {
    compteurTimerFin++;     //en dixième de secondes
    if (compteurTimerFin > 6000) {
      tempsDepasse = true;
    }
    if (compteurTimerFin == 10) {
      _t = true;
    }
  }

  LedToggleTimer4;
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
  int x = 5;
  int y = 5;
  x = analogRead(A3);
  y = analogRead(pinJoystickY);

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
    switch (etat) {
      case ATTENTE : {
          if (js == CLIC) {
            etat = DEMARRAGE_ATTENTE;
            timeStart = millis();
          } else if (js == HAUT) {
            uDisplay = true;
            Kp += 0.1;
          } else if (js == BAS && Kp >= 1) {
            uDisplay = true;
            Kp -= 0.1;
          }
          break;
        }
      case ARRIVE: { 
        if (js == DROITE){
          uDisplay = true;
          if (menuARRIVE < DISTANCEPARCOURUS){
            menuARRIVE = menuARRIVE + 1;
          }
          else{
            menuARRIVE=CONSO;
          }
        }
        else if (js == GAUCHE){
          uDisplay = true;
          if (menuARRIVE > CONSO){
            menuARRIVE = menuARRIVE - 1;
          }
          else{
            menuARRIVE=DISTANCEPARCOURUS;
          }
        }
        if (js == CLIC) {
          uDisplay = true;
          etat = ATTENTE;
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
        switch(menuARRIVE){
          case BRAVO:
          {
            lcd.setRGB(0, 255, 0);
            lcd.print("Bravo M.A.R.K");
            break;
          }
          case CONSO:
          {
            lcd.print("Energie conso : ");
            lcd.setCursor(0, 1);
            lcd.print((Energie*float(compteurTimerFin))/36000.0);
            lcd.print(" Wh");
            break;
          }
          case TPSPARCOURS:
          {
            lcd.print("Tps parcours : ");
            lcd.setCursor(0, 1);
            lcd.print(compteurTimerFin/600);
            lcd.print(" min ");
            lcd.print((compteurTimerFin%600)/10);
            lcd.print(" sec");
            break;
          }
          case VINGTCM:
          {
            lcd.print("Nb Murs < 20cm : ");
            lcd.setCursor(0, 1);
            lcd.print(Nbvingt);
            break;
          }
          case TRROUES:
          {
            lcd.print("Roue Droite:");
            lcd.print(newKnobRight/1200);
            lcd.setCursor(0, 1);
            lcd.print("Roue Gauche:");
            lcd.print(newKnobLeft/1200);
            break;
          }
          case DISTANCEPARCOURUS:
          {
            lcd.print("Distance : ");
            lcd.print((newKnobRight + newKnobLeft)/12733);
            lcd.print(" m");
            break;
          }
        }
        break;

      case AU :
        lcd.clear();
        lcd.setRGB(255, 0, 0);
        switch (codeAU) {
          case DISTANCE: {
              lcd.print("Distance");
              lcd.setCursor(0, 1);
              lcd.print(nomCapteur);
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

void affichebatterie()
{
  cptTension= analogRead(A0);
  cptTension = (3*cptTension*4980/(1023));
 
  affichageTension = (((78*cptTension)-493)/1000);
  affichageTension = (affichageTension/100);
  led=0;
  for(int i = 0; i<affichageTension;i++){
    led |= 1<<i;
  }
  bar.setBits(led);
}

float consommation()
{
  float conso;
  
  nombre++;
  conso = CourantR + CourantG;
  conso = (0.00221*conso-1.12304)+CONSOHORSHACHEUR;
  conso = (conso*7.1*(tempsArret - tempsDemarrage))/(3600);
  moyenne = moyenne + conso;
  conso = moyenne/nombre;
  return conso;  
}

void setup() {

  Serial.begin(9600);

  //moteurs
  pinMode(43, OUTPUT);
  digitalWrite(43, 0);
  initMoteurs();

  EICRB |= (1 << ISC40) + (1 << ISC50); // Interruptions autorisées sur tout les fronts
  EIMSK |= (1 << INT4) + (1 << INT5); // Activation des sorties d'interruption externe (4 et 5)


  //timer 3 à 10Hz  pour l'acquisition des capteurs
  TCCR4A = 0;
  TCCR4B = (1 << CS41) + (1 << CS40);   //prédiv 64
  TCNT4 = 40535;
  TIMSK4 = 1 << TOIE4;    //démarrage


  pinMode(pinTimer4, OUTPUT);
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

  //Batterie
  bar.begin();
  Serial.begin(115200);

  //Ir
  pinMode(pinIR, INPUT_PULLUP);

  etat = ATTENTE;
}

void loop() {
  switch (etat) {
    case INIT : {
        break;
      }
    case ATTENTE : {
        break;
      }
    case DEMARRAGE_ATTENTE : {
        int IRState = digitalRead(pinIR);      // IRState = 1 si en l'air, IRState = 0 si au sol
        Serial.println(IRState);
        if (IRState) {
          timeStart = millis();
        } else {
          if (timeStart + 3000 < millis()) {
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

        if (_t) {
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
        }

        //capteurs ultrason
        //declencher un arret d'urgence si les capteurs sont trop près du mur
        if (USDistFront < USDDISTMIN || USDistLeft < USDDISTMIN || USDistRight < USDDISTMIN) {
          etat = AU;
          codeAU = DISTANCE;
        }
        //incrémenter compteur pour la détection de murs a moins de 20 cm
        if ((USDistFront < USDISTVINGT || USDistLeft < USDISTVINGT || USDistRight < USDISTVINGT) && !FM_Vingt) {
          Nbvingt++;
          FM_Vingt = true;
        }
        else if(USDistFront > USDISTVINGT && USDistLeft > USDISTVINGT && USDistRight > USDISTVINGT){
          FM_Vingt = false;
        }
        
        //bumpers
        if (!BPD || !BPG) {
          etat = AU;
          codeAU = DISTANCE;
        }

        //test courant max
        CourantR = analogRead(pinCurrentRight);
        CourantG = analogRead(pinCurrentLeft);
        Courant = (CourantG < CourantR) ? CourantR : CourantG;
        static unsigned long TempsCourant = millis();

        if (Courant < 650) {
          TempsCourant = millis();
        } else if (TempsCourant + 2000 < millis() ) {
          etat = AU;
          codeAU = COURANT;
        }

        //test gyroscope
        vitesseRotation = gyro.readFloatGyroZ();
        if (vitesseRotation > 200 || vitesseRotation < -200) {
          etat = AU;
          codeAU = ROTATION;
        }

        //gestion du temps de fin
        if (tempsDepasse) {
          etat = TEMPSDEPASSE;
        }

        //deplacement
        if (USDistLeft > USDistRight) {
          //tourne a gauche
          float erreur = (USDistLeft / USDistRight)*Kp;
          int V = 0;

          VMoteurG = Vmax - erreur;
          VMoteurD = Vmax;
        } else if (USDistLeft < USDistRight) {
          //tourne a droite
          float erreur = (USDistRight / USDistLeft)*Kp;
          int V = 0;

          VMoteurG = Vmax;
          VMoteurD = Vmax - erreur;
        } else {
          //tout droit
          VMoteurG = Vmax;
          VMoteurD = Vmax;
        }
        Energie=consommation();  
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
  affichebatterie();
}
