#ifndef SMART_AIRPORT_HPP
#define SMART_AIRPORT_HPP

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "rgb_lcd.h" // Bibliothèque dédiée au Grove LCD RGB Backlight
#include "Adafruit_SHT31.h" // Bibliothèque pour le capteur de température et d'humidité SHT31
#include <PubSubClient.h> // Bibliothèque MQTT pour la communication
#include <stdexcept> // Bibliothèque pour la gestion des exceptions
#include <vector> // Bibliothèque pour l'utilisation des vecteurs

// Fonctions globales
void updateDisplay(); // Met à jour l'affichage
void Initialization(); // Initialise les composants du système
void messageCallback(char* topic, uint8_t* payload, unsigned int length); // Gère les messages MQTT reçus
void Fire_Alarm_Check(); // Vérifie l'état de l'alarme incendie
void Windows_Automatic_Open_Close(); // Gère l'ouverture/fermeture automatique des fenêtres
void Light_Automatic_On_Off(); // Gère l'activation/désactivation automatique des lumières
void Airplane_In_Gate_Check(); // Vérifie si un avion est en position dans la porte
void Wifi_Connected_Check(); // Vérifie si le Wi-Fi est connecté

// Classe pour gérer la connexion Wi-Fi
class WifiManager {
private:
  const char* ssid;       // Nom du réseau Wi-Fi
  const char* password;   // Mot de passe du réseau Wi-Fi

public:
  WifiManager(const char* ssid, const char* password); // Constructeur
  void init(); // Initialise la connexion Wi-Fi
  bool isConnected(); // Vérifie l'état de la connexion Wi-Fi
  String getIP(); // Retourne l'adresse IP actuelle
  void reconnect(); // Réessaie de se connecter au Wi-Fi
};

// Classe pour gérer le capteur de température et d'humidité SHT31
class TemperatureHumiditySensor {
private:
  Adafruit_SHT31 sht31; // Objet Adafruit pour le capteur SHT31
  byte i2cAddress; // Adresse I2C du capteur

public:
  TemperatureHumiditySensor(byte address); // Constructeur
  void init(); // Initialise le capteur
  bool begin(); // Démarre la communication avec le capteur
  float getTemperature(); // Récupère la température en Celsius
  float getHumidity(); // Récupère l'humidité en pourcentage
  void show(); // Affiche les mesures sur un écran
  bool isValidReading(); // Vérifie si les données du capteur sont valides
};

// Classe pour gérer l'écran RGB LCD
class ScreenManager {
private:
  byte SDA, SCL; // Broches I2C pour la communication avec l'écran
  int r, g, b; // Valeurs RGB pour la couleur de l'écran
  String Message1, Message2; // Messages à afficher

public:
  ScreenManager(byte SDA, byte SCL); // Constructeur
  void setrgb(uint8_t r, uint8_t g, uint8_t b); // Définit la couleur de l'écran
  void show(uint8_t r, uint8_t g, uint8_t b, String Message1, String Message2); // Affiche des messages avec une couleur spécifique
  void init(); // Initialise l'écran
  ScreenManager& operator<<(const String& message); // Surcharge de l'opérateur pour ajouter des messages
};

// Classe pour gérer les communications MQTT
class MqttClient {
private:
  const char* server; // Adresse du serveur MQTT
  const int port; // Port du serveur MQTT
  const char* user; // Nom d'utilisateur MQTT
  const char* password; // Mot de passe MQTT
  WiFiClient wifiClient; // Client Wi-Fi
  PubSubClient mqttClient; // Client MQTT

public:
  MqttClient(const char* server, int port, const char* user, const char* password); // Constructeur
  void connectMQTT(); // Connexion au serveur MQTT
  void publishData(const char* topic, float data1, float data2, bool data3); // Publie des données
  void subscribeData(const char* topic, void (*callback)(char*, uint8_t*, unsigned int)); // S'abonne à un topic
  void loop(); // Boucle principale MQTT
};

// Classe de base pour les actionneurs
class Actuator {
protected:
  byte pin; // Broche utilisée par l'actionneur

public:
  Actuator(byte pin); // Constructeur
  virtual void init(); // Initialise l'actionneur (polymorphisme)
  byte getPin() const; // Récupère la broche associée
};

// Classe pour les LED, dérivée d'Actuator
class Led : public Actuator {
public:
  Led(byte pin); // Constructeur
  void on(); // Allume la LED
  void off(); // Éteint la LED
};

// Classe pour les buzzers, dérivée d'Actuator
class Buzzer : public Actuator {
public:
  Buzzer(byte pin); // Constructeur
  void playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns); // Joue un motif d'alarme
  void SetTone(); // Définit une tonalité
  void SetnoTone(); // Arrête la tonalité
};

// Classe pour les moteurs, dérivée d'Actuator
class RoofMotor : public Actuator {
private:
  int angle; // Angle actuel du moteur

public:
  RoofMotor(byte pin); // Constructeur
  void init() override; // Initialise le moteur
  void setAngle(int angle); // Définit un angle spécifique
};

// Classe de base pour les capteurs
class Sensor {
protected:
  byte pin; // Broche associée
  int id; // Identifiant du capteur
  String type; // Type de capteur

public:
  Sensor(int id, String type, byte pin); // Constructeur
  virtual void init(); // Initialise le capteur
  virtual float measure(); // Mesure une valeur (polymorphisme)
  virtual void displayValue(); // Affiche la valeur mesurée
};

// Classe pour le capteur de luminosité, dérivée de Sensor
class LightSensor : public Sensor {
private:
  int lightValue; // Valeur mesurée de la luminosité

public:
  LightSensor(int id, String type, byte pin); // Constructeur
  float measure() override; // Mesure la luminosité
  void displayValue() override; // Affiche la valeur de la luminosité
};

// Classe pour les boutons, dérivée de Sensor
class Button : public Sensor {
private:
  bool Activated; // État du bouton

public:
  Button(int id, String type, byte pin); // Constructeur
  bool IsActivated(); // Vérifie si le bouton est activé
};

// Classe pour les capteurs ultrasoniques, dérivée de Sensor
class UltrasonicSensor : public Sensor {
private:
  long duration; // Durée de l'impulsion
  int distance; // Distance calculée

public:
  UltrasonicSensor(int id, String type, byte pin); // Constructeur
  int measureDistance(); // Mesure la distance
};

// Déclarations externes pour les objets globaux
extern WifiManager wifi; // Gestionnaire Wi-Fi
extern MqttClient mqttClient; // Client MQTT
extern ScreenManager screen; // Gestionnaire d'écran
// Actionneurs
extern Led lamp;
extern Led debugLight;
extern Buzzer alarmBuzzer;
extern RoofMotor roofMotor;
// Capteurs
extern LightSensor luxSensor;
extern Button emergencyButton;
extern Button touchButton;
extern UltrasonicSensor distanceSensor;
extern TemperatureHumiditySensor weatherSensor;

#endif // SMART_AIRPORT_HPP
