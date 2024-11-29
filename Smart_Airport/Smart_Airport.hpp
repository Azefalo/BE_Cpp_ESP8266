#ifndef SMART_AIRPORT_HPP
#define SMARTI_AIRPORT_HPP

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "rgb_lcd.h" // Bibliothèque dédiée au Grove LCD RGB Backlight
#include "Adafruit_SHT31.h"


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

  // Méthode pour allumer la LED
  void on();

  // Méthode pour éteindre la LED
  void off();
};

class Buzzer : public Actuator {
public:
    Buzzer(byte pin); // Le constructeur utilise simplement l'attribut pin de la classe Actuator
    void playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns);
    void SetTone();
    void SetnoTone();
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
  byte pin;
  int id;
  String type;

public:
  Capteur(int id, String type, byte pin);
  virtual void init(); //polymorphisme
  virtual float mesurer(); // Méthode virtuelle pure pour mesurer
  virtual void afficherValeur(); // Pour afficher la valeur sur l'écran
};

class CapteurLuminosite : public Capteur {
private:
  //int pin;
  int valeurLuminosite;

public:
  CapteurLuminosite(int id, String type, byte pin);

  float mesurer();

  void afficherValeur();
};

class Button : public Capteur{
private:
  bool Activated;
public:
  Button(int id, String type, byte pin);
  
  bool IsActivated();
};

// Classe para gerenciar o sensor SHT31
class TemperatureHumiditySensor {
private:
    Adafruit_SHT31 sht31; // Objeto interno da biblioteca Adafruit
    byte i2cAddress;   // Endereço I2C do sensor

public:
  // Construtor para inicializar com o endereço I2C
  TemperatureHumiditySensor(byte address);

  // Inicializa o sensor
  void init();
  bool begin();

  // Retorna a temperatura atual em Celsius
  float getTemperature();

   // Retorna a umidade atual em porcentagem
  float getHumidity();

  void show();

  // Verifica se os valores são válidos
  bool isValidReading();
};

class ScreenManager{
private:
  byte SDA,SCL;
  int r,g,b;
  String Message1,Message2;
  
public : 
  ScreenManager(byte SDA,byte SCL);
  void setrgb(uint8_t r, uint8_t g, uint8_t b);
  void show(uint8_t r , uint8_t g , uint8_t b,String Message1,String Message2);
  void init();
};

class WifiManager {
private:
  const char* ssid;       // Wi-Fi's name
  const char* password;   // Wi-Fi's password

public:
  // Construtor
  WifiManager(const char* ssid, const char* password);

  // Inicializa a conexão Wi-Fi
  void init();

  // Verifica se está conectado
  bool isConnected();

  // Retorna o endereço IP
  String getIP();

  // Reconecta ao Wi-Fi
  void reconnect();
};

#endif // SMART_AIRPORT