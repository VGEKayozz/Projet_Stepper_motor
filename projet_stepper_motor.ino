#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Stepper.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // Adresse 0x27 (voir code "I2C Detector")

int BP_H = 7;       // Horaire BP (mode 2)
int BP_S = 6;       // Stop BP (mode 1)
int BP_A = 5;       // Anti-horaire BP (mode 3)

int in1 = 13;        // Motor IN1
int in2 = 12;        // Motor IN2
int in3 = 11;         // Motor IN3
int in4 = 10;         // Motor IN4
int LedStop = 2;     // LED arrêt
int LedRun = 3;      // LED marche
int potPin = A0;     // Potentiomètre
int buzzerPin = 8;   // Buzzer

const int stepsPerRevolution = 2048; // Nombre de pas pour une révolution complète
Stepper stepper(stepsPerRevolution, in1, in3, in2, in4);

int Mode = 0; // Mode initial (0 = stop, 1 = horaire, 2 = anti-horaire)

void setup() {
  pinMode(BP_H, INPUT_PULLUP);    // Entrée en pull up
  pinMode(BP_S, INPUT_PULLUP);    // Entrée en pull up
  pinMode(BP_A, INPUT_PULLUP);    // Entrée en pull up

  pinMode(LedStop, OUTPUT); // Led arrêt sortie
  pinMode(LedRun, OUTPUT);  // Led marche sortie
  pinMode(buzzerPin, OUTPUT); // Buzzer sortie

  stepper.setSpeed(10); // Vitesse moteur

  lcd.init();          // Allumer l'écran
  lcd.backlight();     // Rétroéclairage de l'écran
  lcd.setCursor(0, 0); // Position du texte
  lcd.print("Projet ");
  lcd.setCursor(0, 1); // Position du texte
  lcd.print("Programmation B1");
  delay(2000);         // Delay pour savoir lire le texte
  lcd.clear();
}

void loop() {
  // Lire l'état des BP pour controle moteur
  if (digitalRead(BP_H) == 0) 
  { Mode = 1; }  // Sens horaire
  if (digitalRead(BP_S) == 0) 
  { Mode = 0; }  // Arrêter le moteur
  if (digitalRead(BP_A) == 0) 
  { Mode = 2; }  // Sens antihoraire

  lcd.setCursor(0, 0);
  if (Mode == 0) { 
    lcd.print("      Arret      ");
    digitalWrite(LedStop, HIGH);
    digitalWrite(LedRun, LOW);
    // Lire la valeur du potentiomètre et ajuster la tonalité du buzzer
    int potValue = analogRead(potPin);       // Lire valeur potentiomètre
    int toneFreq = map(potValue, 0, 1023, 100, 2000); // Convertir en fréquence entre 100 et 2000 Hz
    tone(buzzerPin, toneFreq);               // Jouer la tonalité sur le buzzer
  } else {  
    lcd.print((Mode == 1) ? "    Horaire   " : "  Anti-horaire "); // condition avec le ? horaire = vrai et anti-horraire = faux
    digitalWrite(LedRun, HIGH);
    digitalWrite(LedStop, LOW);
    noTone(buzzerPin);  // Stop buzzer quand moteur tourne

    // Faire tourner le moteur dans le bon sens
    int steps = 10; // Nombre de pas à effectuer par itération
    if (Mode == 1) {
      stepper.step(steps); // Sens horaire
    } else if (Mode == 2) {
      stepper.step(-steps); // Sens antihoraire
    }
  }

  delay(10);
}