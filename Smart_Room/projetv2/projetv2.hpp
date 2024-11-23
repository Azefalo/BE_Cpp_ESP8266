#ifndef PROJETV2_HPP
#define PROJETV2_HPP

#include <Arduino.h>
// Inclut la bibliothèque Servo
#include <Servo.h>

extern Servo servoMotor;

// Définition de la classe de base Actuator
class Actuator {
protected:
    byte pin;  // Pin utilisée par l'actionneur

public:
    // Constructeur pour initialiser la pin de l'actionneur
    Actuator(byte pin);

    // Méthode d'initialisation de la pin en mode OUTPUT
    virtual void init(); //polymorphisme

    // Méthode pour accéder à la pin (si nécessaire pour d'autres actions)
    byte getPin() const;
};

// Définition de la classe Led dérivée de Actuator
class Led : public Actuator {
public:
    // Constructeur de la classe Led qui appelle le constructeur de la classe Actuator
    Led(byte pin);

    // Méthode d'initialisation de la LED
    void init();

    // Méthode pour allumer la LED
    void on();

    // Méthode pour éteindre la LED
    void off();
};

// Définition de la classe servomoteur dérivée de Actuator
class MoteurToit : public Actuator {
private: int angle;
public:
    // Constructeur de la classe servomoteur qui appelle le constructeur de la classe Actuator
    MoteurToit(byte pin);

    // Méthode d'initialisation du moteur
    void init() override;

    // Méthode pour définir l'angle du mote
    void setAngle(int angle);
};


class Capteur {
protected:
    int id;
    String type;

public:
    Capteur(int id, String type);
    virtual float mesurer(); // Méthode virtuelle pure pour mesurer
    virtual void afficherValeur(); // Pour afficher la valeur sur l'écran
};

class CapteurLuminosite : public Capteur {
private:
    int pin;
    int valeurLuminosite;

public:
    CapteurLuminosite(int id, String type,int pin);

    float mesurer();

    void afficherValeur();
};

#endif // PROJET_H
