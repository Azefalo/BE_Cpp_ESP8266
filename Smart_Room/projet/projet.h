#ifndef PROJET_H
#define PROJET_H

#include <Arduino.h>
//
//class Capteur {
//protected:
//    int id;
//    String type;
//
//public:
//    Capteur(int id, String type);
//    virtual float mesurer() = 0; // Méthode virtuelle pure pour mesurer
//    //virtual void afficherValeur() = 0; // Pour afficher la valeur sur l'écran
//};

class Actuator{ // définition de la classe actionneur
  protected : byte pin;
  
  public :
  Actuator(byte pin);
  void init();
};

class Led: public Actuator{
  public : 
  
  Led(byte pin);
  void init();
  void on();
  void off();
  
};

#endif 
