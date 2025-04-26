#include <AccelStepper.h>
#include <HCSR04.h>
#include <Wire.h>
#include <LCD_I2C.h>
#include <U8g2lib.h>

#define CLK_PIN 30
#define DIN_PIN 34
#define CS_PIN 32

U8G2_MAX7219_8X8_F_4W_SW_SPI matrice(U8G2_R0, CLK_PIN, DIN_PIN, CS_PIN, U8X8_PIN_NONE, U8X8_PIN_NONE);
LCD_I2C lcd(0x27, 16, 2);

HCSR04 hc(5, 6);
#define MOTOR_INTERFACE_TYPE 4
#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37
AccelStepper moteur(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

const int pinRouge = 9;
const int pinBleu = 11;
const int pinBuzzer = 8;

enum Etat { FERMEE, TRANSITION, OUVERTE, ALARME };
Etat etatActuel = FERMEE;

const int positionInitiale = 0;
const int positionFinale = 2038;
float distanceAlarme = 15;
float limiteInferieure = 10;
float limiteSuperieure = 60;

float distanceActuelle = 0;
String commandeSerie = "";

unsigned long dernierAffichage = 0;
unsigned long dernierEcho = 0;
unsigned long dernierSon = 0;
unsigned long tempsDebutAlarme = 0;
unsigned long finAffichageTemp = 0;

bool etatBuzzer = false;
bool enAffichageTemporaire = false;

enum AffichageTemp { AUCUN, CHECK, ERREUR, LIMITE_ERR };
AffichageTemp etatMessage = AUCUN;

void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();

  matrice.begin();
  matrice.setContrast(100);

  pinMode(pinRouge, OUTPUT);
  pinMode(pinBleu, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);

  moteur.setMaxSpeed(700);
  moteur.setAcceleration(200);
  moteur.setSpeed(300);
  moteur.setCurrentPosition(positionInitiale);

  Serial.println(F("\n--- Menu disponible ---"));
  Serial.println(F("> gDist             → Affiche la distance mesurée"));
  Serial.println(F("> cfg;alm;X         → Définit distance alarme"));
  Serial.println(F("> cfg;lim_inf;X     → Définit limite inférieure"));
  Serial.println(F("> cfg;lim_sup;X     → Définit limite supérieure"));
  Serial.println(F("-----------------------"));
  Serial.println(F("Entrez une commande :"));
}

void loop() {
  unsigned long maintenant = millis();

  if (maintenant - dernierEcho >= 50) {
    distanceActuelle = hc.dist();
    dernierEcho = maintenant;
  }

  lireCommandeSerie();
  gererEtat(maintenant);

  if (!enAffichageTemporaire && (maintenant - dernierAffichage >= 200)) {
    afficherEtat(distanceActuelle, moteur.currentPosition(), etatActuel == TRANSITION);
    dernierAffichage = maintenant;
  }

  if (enAffichageTemporaire && maintenant >= finAffichageTemp) {
    enAffichageTemporaire = false;
    etatMessage = AUCUN;
    matrice.clearBuffer();
    matrice.sendBuffer();
  }
}

void afficherEtat(float dist, int pos, bool enTransition) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist : ");
  lcd.print(dist, 0);
  lcd.print(" cm");

  lcd.setCursor(0, 1);
  lcd.print("Porte: ");
  if (enTransition) {
    int angle = map(pos, positionInitiale, positionFinale, 10, 170);
    lcd.print(angle);
    lcd.print(" deg");
  } else {
    String etat = (etatActuel == OUVERTE) ? "OUVERTE" : "FERMEE";
    lcd.print(etat);
  }
}

void afficherMatrice(String texte) {
  matrice.clearBuffer();
  matrice.setFont(u8g2_font_5x7_tf);
  matrice.drawStr(0, 7, texte.c_str());
  matrice.sendBuffer();
}

void afficherTemporaire(AffichageTemp type) {
  enAffichageTemporaire = true;
  finAffichageTemp = millis() + 3000;
  etatMessage = type;

  switch (type) {
    case CHECK:       afficherMatrice("V"); break;
    case ERREUR:      afficherMatrice("X"); break;
    case LIMITE_ERR:  afficherMatrice("!"); break;
    default:          matrice.clearBuffer(); matrice.sendBuffer(); break;
  }
}

void gererEtat(unsigned long maintenant) {
  int pos = moteur.currentPosition();

  switch (etatActuel) {
    case FERMEE:
      if (distanceActuelle < distanceAlarme) {
        etatActuel = ALARME;
        tempsDebutAlarme = maintenant;
      } else if (distanceActuelle < limiteSuperieure) {
        etatActuel = TRANSITION;
        moteur.moveTo(positionFinale);
      }
      break;

    case OUVERTE:
      if (distanceActuelle < distanceAlarme) {
        etatActuel = ALARME;
        tempsDebutAlarme = maintenant;
      } else if (distanceActuelle > limiteSuperieure) {
        etatActuel = TRANSITION;
        moteur.moveTo(positionInitiale);
      }
      break;

    case TRANSITION:
      moteur.run();
      if (moteur.distanceToGo() == 0) {
        etatActuel = (pos >= positionFinale - 5) ? OUVERTE : FERMEE;
      }
      break;

    case ALARME:
      if (maintenant - dernierSon >= 250) {
        etatBuzzer = !etatBuzzer;
        digitalWrite(pinBuzzer, etatBuzzer);
        changerCouleur(etatBuzzer ? 255 : 0, etatBuzzer ? 0 : 255);
        dernierSon = maintenant;
      }

      if (distanceActuelle >= distanceAlarme && maintenant - tempsDebutAlarme >= 3000) {
        digitalWrite(pinBuzzer, LOW);
        changerCouleur(0, 0);
        etatActuel = FERMEE;
      }
      break;
  }
}

void changerCouleur(int rouge, int bleu) {
  analogWrite(pinRouge, rouge);
  analogWrite(pinBleu, bleu);
}

void lireCommandeSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      commandeSerie.trim();
      traiterCommande(commandeSerie);
      commandeSerie = "";
    } else {
      commandeSerie += c;
    }
  }
}

void traiterCommande(String commande) {
  commande.trim();
  commande.toLowerCase();

  if (commande == "gdist") {
    Serial.print("Distance = ");
    Serial.print(distanceActuelle, 1);
    Serial.println(" cm");
    afficherTemporaire(CHECK);
    return;
  }

  if (commande.startsWith("cfg;alm;")) {
    String valStr = commande.substring(8);
    float val = valStr.toFloat();
    if (val > 0) {
      distanceAlarme = val;
      Serial.print("Alarme definie à : ");
      Serial.println(val);
      afficherTemporaire(CHECK);
    } else {
      Serial.println("Valeur invalide pour alarme.");
      afficherTemporaire(ERREUR);
    }
  } else if (commande.startsWith("cfg;lim_inf;")) {
    String valStr = commande.substring(12);
    float val = valStr.toFloat();
    if (val >= limiteSuperieure) {
      Serial.println("Erreur : lim_inf >= lim_sup");
      afficherTemporaire(LIMITE_ERR);
    } else {
      limiteInferieure = val;
      Serial.print("lim_inf definie à : ");
      Serial.println(val);
      afficherTemporaire(CHECK);
    }
  } else if (commande.startsWith("cfg;lim_sup;")) {
    String valStr = commande.substring(12);
    float val = valStr.toFloat();
    if (val <= limiteInferieure) {
      Serial.println("Erreur : lim_sup <= lim_inf");
      afficherTemporaire(LIMITE_ERR);
    } else {
      limiteSuperieure = val;
      Serial.print("lim_sup definie à : ");
      Serial.println(val);
      afficherTemporaire(CHECK);
    }
  } else {
    Serial.println("Commande inconnue !");
    afficherTemporaire(ERREUR);
  }

  Serial.println("Entrez une commande :");
}
