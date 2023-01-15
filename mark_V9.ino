/**
 * @file mark_V9.ino
 * @author DE JAGER Thibault, DUPUY Jules, LE CLEVE Axel, HAMON Dimitri (Nantes université)
 * @brief (French) Ce programme à été créé suite à un projet pédagogique proposé par Polytech Nantes dans le cadre de la première année de la formation CCSE
Il a pour but de contrôler un robot M.A.R.K. (arduino Mega2560) et lui permettre de se déplacer d'un départ à une arrivé donné.
Il fait donc l'acquisition de différents capteurs (ultrasons, infra-rouge, encodeur de roue).
Il contrôle 2 moteurs pas à pas par MLI (PWM).
 * @version 9.0
 * @date 2023-01-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "Ultrasonic.h"
#include <Encoder.h>
#include <Grove_LED_Bar.h>
#include "rgb_lcd.h"
#include "SparkFunLSM6DS3.h"
#include "Wire.h"


//définition des pins des capteurs
#define pinUSFront        8
#define pinUSLeft         12
#define pinUSRight        10
#define pinIR             6
#define pinBpG            2
#define pinBpD            3
#define pinKnobLeft       18,29
#define pinKnobRight      19,27
#define pinCurrentLeft    A5
#define pinCurrentRight   A4
#define pinJoystickX      A3
#define pinJoystickY      A2

//visualisation des ISR
#define pinTimer4         7
#define pinTimer5         13
#define LedToggleTimer4   digitalWrite(pinTimer4, !digitalRead(pinTimer4))
#define LedToggleTimer5   digitalWrite(pinTimer5, !digitalRead(pinTimer5))


//seuils détection d'erreur
#define DELTAKNOB 5             //plage d'erreur pour le blocage de roue
#define TEMPS_PARCOURS_MAX 6000 //temps de parcour max à partir duquel le robot s'arrête (en dizième de secondes)
#define USDDISTMIN 10           //Distance de déclenchement de l'arrêt d'urgence (en centimètres)
#define CONSOHORSHACHEUR 0.55   //550mA de conso sans les moteurs
#define USDISTVINGT 20          //Distance d'incrémentation du compteur de proximité avec un mur


//gestion des coéficients de déplacement
//pour l'étape 1
#define KPG 2
#define KPD 17

//pour l'étape 2
#define KPG2 20
#define KPD2 13

//pour l'étape 3
#define KPG5 13
#define KPD5 10

//Distances entre le robot et le mur demandé 
#define ASSERVISSEMENTETAPE1 75
#define ASSERVISSEMENTETAPE3 65
#define ASSERVISSEMENTETAPE5 50

//Valeur de diminution de la roue a l'interieur d'un virage (valeur ++ --> vitesse ++)
#define KPVIRAGEGAUCHE 120
#define KPVIRAGEDROITE 80

//seuils pour transitions entre étapes (déplacement)
#define DISTANCEFRONTETAPE1 100
#define DISTANCEFRONTETAPE2 400
#define DISTANCEGAUCHE 160
#define DISTANCEDROITE 125
#define TEMPO 2

//define moteur
#define Thash 800
#define Stop 400
#define Vmax 700



// Macros Moteur / ne pas modifier
#define MoteurG(Vg) OCR5A=Vg // Vg in [0... 1999]
#define MoteurD(Vd) OCR5B=Vd // VD in [0... 1999]
#define MoteurGD(Vg,Vd) MoteurG(Vg);MoteurD(Vd)
#define StopMoteurGD MoteurGD(Stop,Stop)

//Gestion Etat du Robot

//machine d'état principale du robot
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

enum MenuARRIVE {
  BRAVO,
  CONSO,
  TPSPARCOURS,
  VINGTCM,
  TRROUES,
  DISTANCEPARCOURUS
};

enum Etapes {
  ETAPE1,
  ETAPE2,
  ETAPE3,
  ETAPE3_TEMPO,
  ETAPE4,
  ETAPE5
};

int etape3Tempo = 0;    //temporisation de 1s entre l'étape 3 et 4


String nomCapteur;

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
volatile enum Etapes etape = ETAPE1;

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
volatile long  cptTension = 0;
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


volatile bool detectionBlocageRoue = false;         //est mise à vrai 1 seconde après le démarrage du robot, elle permet d'empêcher une fausse détéction de roue bloqué 
volatile bool uDisplay = false;   //est mise à 1 si on veut mettre à jour l'affichage LCD

//Fonction d'initialisation du timer gérant la MLI contrôlant les moteurs
void initMoteurs() {    // MoteurG :OC5A=PIN46-PL3, MoteurD : OC5B=PIN45-PL4
  DDRL = 0x18 ;         // PL3 et PL4
  DDRB = 0x80 ;         // PB7 LedToggle

  // COM5B_1:0 = 10   -> clear sur egalite++, set sur egalite--
  // WGM5_3:1 = 1000  -> mode 8 => ICR5 defini le TOP
  TCCR5A = (1 << COM5A1) + (1 << COM5B1);
  TCCR5B = (1 << ICNC5) + (1 << WGM53) + (1 << CS50); // CS_12:10  = 001  -> prediv par 1
  ICR5 = Thash;           // 1999 correspond a f = 4khz
  StopMoteurGD;
  TIMSK5 = 1 << TOIE5;    // Interrution de débordement du timer 
}


//Code executé lors d'une interruption du timer5
ISR(TIMER5_OVF_vect) {
  LedToggleTimer5;
  IRState = digitalRead(pinIR);
  LedToggleTimer5;
}

//Code executé lors d'une interruption du timer4
//L'acquisition des capteur est effectué ici
ISR (TIMER4_OVF_vect) {
  LedToggleTimer4;
  TCNT4 = 40535; //40535

  //on utilise des compteurs pour diminuer la fréquence d'acquisition de certain capteurs
  static int compteurKnob_T5 = 1;
  static int compteurUS_T5 = 0;

  //l'acquisition des capteurs n'est effectué que lors du démarage et de déplacement du robot
  if (etat == MARCHE || etat == DEMARRAGE_ATTENTE || etat == DEMARRAGE) {
    compteurKnob_T5++;
    compteurUS_T5++;

    // Interruption pour les ultrasons
    if (compteurUS_T5 == 2) {
      compteurUS_T5 = 0;
      USDistFront = USFront.MeasureInCentimeters();
      USDistRight = USRight.MeasureInCentimeters();
      USDistLeft = USLeft.MeasureInCentimeters();
    }

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

  //on incrémente le compteur de temps de parcour uniquement lors du déplacement du robot
  if (etat == MARCHE) {
    compteurTimerFin++;     //en dixième de secondes
    if (compteurTimerFin > TEMPS_PARCOURS_MAX) {  //détection de dépassement du temps de parcour max
      tempsDepasse = true;
    }
    if (compteurTimerFin == 10) {   //activation de la détection roue bloqué
      detectionBlocageRoue = true;
    }
    uDisplay = true;
    if (compteurTimerFin % 10 == 0) {
    }
  }

  LedToggleTimer4;
}


//Cette fonction permet d'obtenir l'état actuelle du joystick
//Elle renvoie une valeur d'un énum : REPOS, HAUT, BAS, DROITE, GAUCHE, CLIC
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


/* Cette fonction est appellé dans la boucle principale
 * Elle permet de gérer le comportement du robot en fonction de l'état actuel du robot
 * Exemple : 
  - changer de menu a afficher à l'arrivé du robot
  - démarrer le robot
 */
void updateControls() {
  static JoystickState jsPrec;
  JoystickState js = getJoystickState();
  if (jsPrec != js) {
    jsPrec = js;
    switch (etat) {
      case ATTENTE : {
          if (js == CLIC) {
            etat = DEMARRAGE_ATTENTE;
          }
          break;
        }

      case ARRIVE: {
          if (js == DROITE) {
            uDisplay = true;
            if (menuARRIVE < DISTANCEPARCOURUS) {
              menuARRIVE = menuARRIVE + 1;
            }
            else {
              menuARRIVE = CONSO;
            }
          }
          else if (js == GAUCHE) {
            uDisplay = true;
            if (menuARRIVE > CONSO) {
              menuARRIVE = menuARRIVE - 1;
            }
            else {
              menuARRIVE = DISTANCEPARCOURUS;
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

/* Cette fonction permet de gérer l'affichage sur l'écran LCD
 * le changement de l'affichage du lcd s'effectue lors d'un changement d'état du robot ou lors de la mise à vrai de la variable uDisplay
*/
void updateDisplay() {
  static Etat etatPrec = INIT;
  Etat e = etat;
  if (etatPrec != e || uDisplay == true) {
    uDisplay = false;
    etatPrec = etat;
    switch (etat) {
      case INIT :
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("INITIALISATION");
        break;

      case ATTENTE:
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("   EN ATTENTE   ");
        break;
      case DEMARRAGE_ATTENTE :
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write(" POSER LE ROBOT ");
        lcd.setCursor(0, 1);
        lcd.write("  ATTENDEZ 3S   ");
        break;
      case DEMARRAGE:
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("   DEMARRAGE    ");
        lcd.setCursor(0, 1);
        lcd.write("    EN COURS    ");
        break;

      case MARCHE:
        lcd.clear();
        lcd.setRGB(255, 255, 255);
        lcd.setCursor(0, 0);
        lcd.write("temps :");
        lcd.setCursor(0, 1);
        lcd.print(compteurTimerFin / 600);
        lcd.print(" min ");
        lcd.print( float(compteurTimerFin % 600) / 10);
        lcd.print(" sec");
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
        switch (menuARRIVE) {
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
              lcd.print((Energie * float(compteurTimerFin)) / 36000.0);
              lcd.print(" Wh");
              break;
            }
          case TPSPARCOURS:
            {
              lcd.print("Tps parcours : ");
              lcd.setCursor(0, 1);
              lcd.print(compteurTimerFin / 600);
              lcd.print(" min ");
              lcd.print( float(compteurTimerFin % 600) / 10);
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
              lcd.print(newKnobRight / 1200);
              lcd.setCursor(0, 1);
              lcd.print("Roue Gauche:");
              lcd.print(newKnobLeft / 1200);
              break;
            }
          case DISTANCEPARCOURUS:
            {
              lcd.print("Distance : ");
              lcd.setCursor(0, 1);
              lcd.print((newKnobRight + newKnobLeft) / 127);
              lcd.print(" cm");
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

//Cettte fonction gère le fonctionnement de l'afficheur de batterie 
void affichebatterie() {
  cptTension = analogRead(A0);
  cptTension = (3 * cptTension * 4980 / (1023));

  affichageTension = (((78 * cptTension) - 493) / 1000);
  affichageTension = (affichageTension / 100);
  led = 0;
  for (int i = 0; i < affichageTension; i++) {
    led |= 1 << i;
  }
  bar.setBits(led);
}

//Cette fonction renvoie la consomation moyenne a chaques appels 
float consommation() {
  float consoR;
  float consoG;
  float consoT;

  nombre++;
  consoR = CourantR;
  consoR = (0.00221 * consoR - 1.12304);
  consoG = CourantG;
  consoG = (0.00221 * consoG - 1.12304);
  consoT = consoR + consoG + CONSOHORSHACHEUR;
  consoT = (consoT * 7.1 * (tempsArret - tempsDemarrage)) / (3600);
  moyenne = moyenne + consoT;
  consoT = moyenne / nombre;
  return consoT;
}


/* Fonction appellé à la mise sous tension du robot
 * Elle permet de paramètrer les mods des pins et les différents timers
 * y sont aussi paramètré les diférents modules (capteurs ultrason, ...)
*/
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

//Gestion des transitions entre les différentes étapes de déplacement
void checkTransition() {
  Serial.println(USDistFront);
  switch (etape) {
    case ETAPE1 : //le robot longe le mur de droite jusqu'à la detection du mur frontal
      if (USDistFront < DISTANCEFRONTETAPE1) {
        etape = ETAPE2;
      }
      break;
    case ETAPE2 : //le robot tourne vers la gauche jusqu'à la détection du couloir 
      if (USDistFront > DISTANCEFRONTETAPE2) {
        etape = ETAPE3;
      }
      break;
    case ETAPE3 : //le robot longe le mur de droite jusqu'à la détection du couloir a droite 
      if (USDistRight > DISTANCEDROITE) {
        etape = ETAPE3_TEMPO;
        etape3Tempo = compteurTimerFin + TEMPO;
      }
      break;
    case ETAPE3_TEMPO : //le robot continue tout droit jusqu'à la fin de la temporisation
      if (compteurTimerFin > etape3Tempo) {
        etape = ETAPE4;
      }
    case ETAPE4 : //le robot tourne a droite jusqu'à la détection du couloir a gauche
      if (USDistLeft > DISTANCEGAUCHE) {
        etape = ETAPE5;
      }
      break;
      //gestion de la detection de la ligne d'arrivée dans la boucle principale
  }
}


//boucle principale du programme
void loop() {
  switch (etat) {     //diagrame d'état du robot
    case INIT : {     //mise sous tension du robot
        break;
      }

    case ATTENTE : {  //attente de l'appuis sur le joystick par l'utilisateur
        break;
      }

    case DEMARRAGE_ATTENTE : {  //le robot attend d'être au sol pendant au moins 3 secondes
        IRState = digitalRead(pinIR);      // IRState = 1 si en l'air, IRState = 0 si au sol
        if (IRState) {
          timeStart = millis();
        } else {
          if (timeStart + 3000 < millis()) {
            etat = DEMARRAGE;
          }
        }
        break;
      }

    case DEMARRAGE : {    //une fois l'attente terminé, le robot démarre
        tempsDepasse = false;
        compteurTimerFin = 0;
        etat = MARCHE;
        etape = ETAPE1;
        break;
      }

    case MARCHE: {
        //acquisition capteur infra rouge pour l'arrivée
        if (IRState) {
          etat = ARRIVE;
        }


        if (detectionBlocageRoue) {
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
        else if (USDistFront > USDISTVINGT && USDistLeft > USDISTVINGT && USDistRight > USDISTVINGT) {
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

        checkTransition();

        //deplacement
        switch (etape) {    //les différentes étapes correspondent au déroulement du parcours
          case ETAPE1:
            if (USDistRight < ASSERVISSEMENTETAPE1) {
              //tourne a gauche
              VMoteurG = Vmax - KPG;
              VMoteurD = Vmax - 7;
            } else {
              VMoteurG = Vmax ;
              VMoteurD = Vmax - KPD;
            }
            break;

          case ETAPE2:
            VMoteurG = Stop;
            VMoteurD = Stop + KPVIRAGEGAUCHE;
            break;

          case ETAPE3:
            if (USDistRight < ASSERVISSEMENTETAPE3) {
              //tourne a gauche
              VMoteurG = Vmax - KPG2;
              VMoteurD = Vmax - 7;
            } else {
              VMoteurG = Vmax ;
              VMoteurD = Vmax - KPD2;
            }
            break;
          case ETAPE3_TEMPO :
            VMoteurG = Vmax;
            VMoteurD = Vmax;
            break;
          case ETAPE4:
            VMoteurG = Stop + KPVIRAGEDROITE;
            VMoteurD = Stop;
            break;

          case ETAPE5 :
            if (USDistRight < ASSERVISSEMENTETAPE5) {
              //tourne a gauche
              VMoteurG = Vmax - KPG5;
              VMoteurD = Vmax - 7;
            } else {
              VMoteurG = Vmax ;
              VMoteurD = Vmax - KPD5;
            }
        }

        break;
      }

    case AU: {
        //affichage de l'arret d'urgence
        VMoteurG = Stop;
        VMoteurD = Stop;
        break;
      }
      
    case TEMPSDEPASSE: {
        break;
      }

    case ARRIVE: {
        VMoteurG = Stop;
        VMoteurD = Stop;
        break;
      }
  }
  MoteurGD(VMoteurG, VMoteurD);
  updateDisplay();
  updateControls();
  affichebatterie();
  Energie = consommation();
}
