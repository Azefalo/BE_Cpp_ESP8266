#include"projetv1.hpp"

//class Capteur {
//protected:
//    int id;
//    String type;
//
//public:
//    Capteur(int id, String type){
//    this->id=id;
//    this->type=type;
//    } 
//    virtual float mesurer() = 0; // Méthode virtuelle pure pour mesurer
//    //virtual void afficherValeur() = 0; // Pour afficher la valeur sur l'écran
//};

// Constructeur de la classe Actuator
Actuator::Actuator(byte pin) {
    this->pin = pin;
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
Capteur :: Capteur(int id, String type){
    this->id=id;
    this->type=type;
} 
virtual float Capteur :: mesurer()=0;
virtual void Capteur :: afficherValeur()=0;


//définition du capteur de luminosité
CapteurLuminosite :: CapteurLuminosite(int id, int pin) : Capteur(id, "Luminosité"){};

float CapteurLuminosite :: mesurer(){
        valeurLuminosite = analogRead(pin);
        return valeurLuminosite;
}

 void CapteurLuminosite :: afficherValeur(){
        Serial.print("Luminosité : ");
        Serial.println(valeurLuminosite);
 }
 
