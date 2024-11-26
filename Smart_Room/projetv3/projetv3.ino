#include "projetv3.hpp"

UltrasonicSensor ultrasonic(7,"Ultrassonic",D7); // Création d'un objet pour le capteur connecté à D1

void setup() {
    Serial.begin(9600); // Initialise la communication série
    Serial.println("Test du capteur ultrasonique avec classe");
    ultrasonic.init(); // Initialise le capteur
}

void loop() {
  int distance = ultrasonic.measureDistance();
  if (distance == -1) {
    Serial.println("Aucun signal reçu ou écho trop long.");
  } else {
    Serial.print("Distance : ");
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(500); // Pause de 500 ms avant la prochaine mesure
}
