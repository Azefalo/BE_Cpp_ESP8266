#include"projetv1.hpp"

// Constructeur de la classe Actuator
Actuator::Actuator(byte pin):pin(pin){
}

// Méthode d'initialisation de la pin en mode OUTPUT
void Actuator::init() {
    pinMode(pin, OUTPUT);
}

// Méthode pour accéder à la pin
byte Actuator::getPin() const {
    return pin;
}

// Constructeur de la classe Led qui appelle le constructeur de la classe Actuator
Led::Led(byte pin) : Actuator(pin) {}

// Méthode d'initialisation de la LED
void Led::init() {
    Actuator::init();  // Appel de l'init de la classe de base
}

// Méthode pour allumer la LED
void Led::on() {
    digitalWrite(getPin(), HIGH);  // Utilisation de getPin() pour récupérer la pin
}

// Méthode pour éteindre la LED
void Led::off() {
    digitalWrite(getPin(), LOW);  // Utilisation de getPin() pour récupérer la pin
}




//définition de la classe Capteur
Capteur :: Capteur(int id, String type):id(id),type(type){
} 

float Capteur :: mesurer(){
return 0;
}
void Capteur :: afficherValeur(){
}


//définition du capteur de luminosité
CapteurLuminosite :: CapteurLuminosite(int id, String type, int pin): Capteur(id, type), pin(pin), valeurLuminosite(0) {}

float CapteurLuminosite :: mesurer(){
        valeurLuminosite = analogRead(pin);
        return valeurLuminosite;
}

 void CapteurLuminosite :: afficherValeur(){
        Serial.print("Luminosité : ");
        Serial.println(valeurLuminosite);
 }
 
