#include "projetv6.hpp"

// Création d'un objet pour le capteur de température et d'humidité
TemperatureHumiditySensor sensor(0x44);

void setup() {
  Serial.begin(9600);  // Initialise la communication série
  Serial.println("Test du Grove - Capteur de température et d'humidité (SHT31)");

  sensor.init(); // Initialisation du capteur
}

void loop() {
  // Vérifie si la lecture du capteur est valide et affiche les données
  sensor.show();

  delay(2000);  // Attente de 2 secondes avant la prochaine lecture
}
