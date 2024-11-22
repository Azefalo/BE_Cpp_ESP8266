#include"projetv1.hpp"

const int lightSensorPin = A0; // Pin analogique connecté au capteur
int sensorValue = 0;          // Variable pour stocker la lecture du capteur

void setup() {
  Serial.begin(9600); // Initialise la communication série
}

void loop() {
  CapteurLuminosite lux(1,"Luminosité",A0);
  lux.mesurer();
  lux.afficherValeur();
  delay(500); // Petite pause pour éviter une lecture continue trop rapide
}
