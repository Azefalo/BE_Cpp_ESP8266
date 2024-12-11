#include "projetv2.hpp"
// Broche de contrôle du servo
const int servoPin = D3;

// Variables pour le contrôle de l'angle
int angle = 0;       // Angle initial
int step = 10;       // Incrément de l'angle

MoteurToit Moteur(D6);
void setup() {
  Moteur.init();
  
  // Initialise la communication série pour fournir des retours
  Serial.begin(115200);
  Serial.println("Test du moteur servo démarré.");
}

void loop() {
  // Fait tourner le servo de 0 à 180 degrés
  for (angle = 0; angle <= 180; angle += step) {
    Moteur.setAngle(angle);      // Définit l'angle du servo
    Serial.print("Angle : ");      // Retour via la communication série
    Serial.println(angle);
    delay(500);                    // Attend pour permettre au servo de bouger
  }

  // Fait tourner le servo de 180 à 0 degrés
  for (angle = 180; angle >= 0; angle -= step) {
    Moteur.setAngle(angle);        // Définit l'angle du servo
    Serial.print("Angle : ");      // Retour via la communication série
    Serial.println(angle);
    delay(500);                    // Attend pour permettre au servo de bouger
  }
}
