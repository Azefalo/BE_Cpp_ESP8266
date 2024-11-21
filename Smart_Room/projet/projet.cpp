//#include"projet.h"
#include <Arduino.h>
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

class Actuator{ // définition de la classe actionneur
  protected : byte pin;
  
  public :
  Actuator(byte pin){
  this->pin=pin;
  }
  void init(){
  pinMode(pin,OUTPUT);
  }
};

class Led: public Actuator{
  private: byte pin;
  
  public : 
  
  Led(byte pin):Actuator(pin){
  }
  void init(){
  Actuator::init();
  }
  void on(){
  digitalWrite(pin,HIGH);
  }
  void off(){
  digitalWrite(pin,LOW);
  }
  
};



//class CapteurLuminosite : public Capteur {
//private:
//    int pin;
//    int valeurLuminosite;
//
//public:
//    CapteurLuminosite(int id, int pin) : Capteur(id, "Luminosité"), pin(pin) {}
//
//    float mesurer() override {
//        valeurLuminosite = analogRead(pin);
//        return valeurLuminosite;
//    }
//
//    void afficherValeur() override {
//        Serial.print("Luminosité : ");
//        Serial.println(valeurLuminosite);
//    }
//};
