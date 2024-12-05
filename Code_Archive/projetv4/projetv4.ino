#include "projetv4.hpp"


PushButton EmergencyButton(5,"Button d'emergence", D7);
Led Light(LED_BUILTIN_AUX);

void setup() {
  Serial.begin(9600); // Initialise la communication série
  Serial.println("Test du button poussoir");
  EmergencyButton.init(); // Initialise le capteur
  Light.init();
}

void loop() {
  if (EmergencyButton.IsActivated() == true){
    Light.on();
  }
  else{
    Light.off();
  delay(300);
  }
}
