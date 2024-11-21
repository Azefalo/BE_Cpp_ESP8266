#include <Arduino.h>

// Définition de la classe de base Actuator
class Actuator {
protected:
    byte pin;  // Pin utilisée par l'actionneur

public:
    // Constructeur pour initialiser la pin de l'actionneur
    Actuator(byte pin) {
        this->pin = pin;
    }

    // Méthode d'initialisation de la pin en mode OUTPUT
    void init() {
        pinMode(pin, OUTPUT);
    }

    // Méthode pour accéder à la pin (si nécessaire pour d'autres actions)
    byte getPin() const {
        return pin;
    }
};

// Définition de la classe Led dérivée de Actuator
class Led : public Actuator {
public:
    // Constructeur de la classe Led qui appelle le constructeur de la classe Actuator
    Led(byte pin) : Actuator(pin) {}

    // Méthode d'initialisation de la LED
    void init() {
        Actuator::init();  // Appel de l'init de la classe de base pour initialiser la pin
    }

    // Méthode pour allumer la LED
    void on() {
        digitalWrite(getPin(), HIGH);  // Utilisation de getPin() pour récupérer la pin
    }

    // Méthode pour éteindre la LED
    void off() {
        digitalWrite(getPin(), LOW);  // Utilisation de getPin() pour récupérer la pin
    }
};

void setup() {
  // Initialisation de la communication série pour afficher les messages
  Serial.begin(9600);


}

void loop() {
    // Création d'un objet Led avec la pin 13 (souvent la LED intégrée sur les cartes Arduino)
  Led led(D4);

  // Initialisation de la LED
  led.init();
  // Allumage de la LED pendant 1 seconde
  led.on();
  delay(1000);  // Attendre 1 seconde

  // Extinction de la LED pendant 1 seconde
  led.off();
  delay(1000);  // Attendre 1 seconde
}
