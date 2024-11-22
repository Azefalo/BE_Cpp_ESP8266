#ifndef PROJET_H
#define PROJET_H

#include <Arduino.h>


class Capteur {
protected:
    int id;
    String type;

public:
    Capteur(int id, String type);
    //virtual float mesurer(); // Méthode virtuelle pure pour mesurer
    //virtual void afficherValeur() = 0; // Pour afficher la valeur sur l'écran
};

// Définition de la classe de base Actuator
class Actuator {
protected:
    byte pin;  // Pin utilisée par l'actionneur

public:
    // Constructeur pour initialiser la pin de l'actionneur
    Actuator(byte pin);

    // Méthode d'initialisation de la pin en mode OUTPUT
    void init();

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

#endif // PROJET_H
