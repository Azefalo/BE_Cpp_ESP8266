#include "Smart_Airport.hpp"
#include "Smart_Airport.ino.globals.h"
#include "credentials.hpp"
#include "ESP8266_Pins.hpp"
#include <Servo.h>

// Global Variables
extern String ATC_Message = "";
rgb_lcd lcd; // Initialisation de l'écran Grove LCD
Servo servoMotor;
std::vector<Capteur*> sensors = {&lux, &emergencyButton, &touchButton};
std::vector<Actuator*> actuators = {&lamp, &debugLight, &alarmBuzzer, &moteur};
bool AlarmActivated = false; 
bool LampActivated = false; // Variable globale pour suivre l'état de la lampe
bool AirplaneInGate = false;

// Function Prototypes
void messageCallback(char* topic, uint8_t* payload, unsigned int length);
void Inicialization();
void Fire_Alarm_Check();
void Windows_Automatic_Open_Close();
void Light_Automatic_On_Off();
void updateDisplay();
void Airplane_In_Gate_Check();
void Wifi_Conected_Check();

// Class Implementations

// WifiManager
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnecting to Wi-Fi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > 10000) { // Timeout de 10 secondes
      throw std::runtime_error("Wi-Fi connection timed out.");
    }
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

bool WifiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

String WifiManager::getIP() {
  if (isConnected()) {
    return WiFi.localIP().toString();
  } else {
    return "\nNot connected";
  }
}

void WifiManager::reconnect() {
  if (!isConnected()) {
    Serial.println("Reconnecting to Wi-Fi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nReconnected to Wi-Fi.");
  }
}

// ScreenManager
ScreenManager::ScreenManager(byte SDA, byte SCL) : SDA(SDA), SCL(SCL) {
  Wire.begin(SDA, SCL);
}

void ScreenManager::init() {
  lcd.begin(16, 2);
  ScreenManager::show(255, 128, 0, "Initialising ...", "Please Wait");
}

void ScreenManager::show(uint8_t r, uint8_t g, uint8_t b, String Message1="", String Message2="") {
  lcd.clear();
  ScreenManager::setrgb(r, g, b);
  lcd.setCursor(0, 0); // Ligne 0, Colonne 0
  lcd.print(Message1);
  lcd.setCursor(0, 1); // Ligne 0, Colonne 1
  lcd.print(Message2);
  delay(1000);
}

void ScreenManager::setrgb(uint8_t r, uint8_t g, uint8_t b) {
  lcd.setRGB(r, g, b);
}

ScreenManager& ScreenManager::operator<<(const String& message) {
  show(255, 255, 0, "     Update     ", message); // Affiche le message sur la ligne 1
  return *this;
}

// MqttClient
MqttClient::MqttClient(const char* server, int port, const char* user, const char* password)
  : server(server), port(port), user(user), password(password), mqttClient(wifiClient) {
  mqttClient.setServer(server, port);
}

void MqttClient::connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Connexion au serveur MQTT...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), user, password)) {
      Serial.println("Connecté au serveur MQTT !");
    } else {
      throw std::runtime_error("Mqtt connection timed out.");
      delay(5000);
    }
  }
}

void MqttClient::publishData(const char* topic, float data1, float data2, bool data3) {
  char payload[50];
  snprintf(payload, sizeof(payload), "%.2f/%.2f/%d", data1, data2, data3 ? 1 : 0);
  mqttClient.publish(topic, payload);
  Serial.print("Publié sur ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(payload);
}

void MqttClient::subscribeData(const char* topic, void (*callback)(char*, uint8_t*, unsigned int)) {
  mqttClient.setCallback(callback);
  mqttClient.subscribe(topic);

  Serial.print("Abonné au sujet : ");
  Serial.println(topic);
}

void MqttClient::loop() {
  mqttClient.loop();
}

// Sensor and Actuator Classes
// ... [Classes Capteur, CapteurLuminosite, Button, UltrasonicSensor, Actuator, Led, Buzzer, MoteurToit]

// Callback Function
void messageCallback(char* topic, uint8_t* payload, unsigned int length) {
  Serial.print("Message reçu sur le sujet : ");
  Serial.println(topic);
  // On lit la charge utile et on l'interprète
  String receivedMessage = "";
  for (unsigned int i = 0; i < length; i++) {
    receivedMessage += (char)payload[i];
  }
  
  Serial.print("Message : ");
  Serial.println(receivedMessage);
  
  // Si le message est "1", on active l'alarme
  if (receivedMessage == "1") {
    AlarmActivated = false;
  } else {
    ATC_Message = receivedMessage;
    screen << receivedMessage;
  }
}

// Function Implementations

void Inicialization() {
  screen.init();

  // Initialise tous les capteurs
  for (auto sensor : sensors) {
    sensor->init();
  }
  // Initialise tous les actionneurs
  for (auto actuator : actuators) {
    actuator->init();
  }
  
  try {
    wifi.init();         // Initialise le Wi-Fi
    weatherSensor.init();    // Inicialises le capteur météo
    distanceSensor.init();
    // Connexion au serveur MQTT
    mqttClient.connectMQTT();
  } catch (const std::runtime_error& e) {
    Serial.println(e.what());
  }
  mqttClient.subscribeData("alarmstop", messageCallback);
}

void Fire_Alarm_Check() {
  // Maintenir la connexion MQTT active et vérifier le payload
  mqttClient.loop(); 
  mqttClient.publishData("data", weatherSensor.getTemperature(), weatherSensor.getHumidity(), AlarmActivated);
  if (emergencyButton.IsActivated() == true) {
    AlarmActivated = true;
    while (AlarmActivated == true) {
      mqttClient.loop(); 
      mqttClient.publishData("data", weatherSensor.getTemperature(), weatherSensor.getHumidity(), AlarmActivated);
      screen.show(255, 16, 0, "Atention! fire", "alarm activated");
      alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
    }
    screen.show(0, 255, 0, "Fire alarm", "Desactivated");
    delay(1000);
  }
}

void Windows_Automatic_Open_Close() {
  // Fonction qui lit le capteur de lumière et règle l'angle du moteur
  moteur.setAngle(lux.mesurer() / 10);
}

void Light_Automatic_On_Off() {
  // Lit la valeur du capteur de luminosité
  int lightLevel = lux.mesurer();
  
  // Vérifie les conditions pour activer ou désactiver la lampe
  if (lightLevel < 200 || touchButton.IsActivated()) {
    if (!LampActivated) { // Ne s'allume que si elle n'est pas encore allumée
      LampActivated = true;
      lamp.on();
    }
  } else if (LampActivated && !touchButton.IsActivated() && lightLevel > 200) {
    LampActivated = false; // Ne s'éteint que si elle était allumée
    lamp.off();
  }
}

void updateDisplay() {
  String temperature = String(weatherSensor.getTemperature(), 1);
  String humidity = String(weatherSensor.getHumidity(), 1);

  // Construire la deuxième ligne de texte
  String infoLine = "T:" + temperature + "oC H:" + humidity + "%";

  // Afficher les informations sur l'écran
  screen.show(255, 255, 255, ATC_Message, infoLine);
}

void Airplane_In_Gate_Check() {
  // Lit la distance du capteur
  int measuredDistance = distanceSensor.measureDistance();
  updateDisplay();
  // Vérifie si l'avion est à la porte (distance entre 5 et 60 cm)
  if (measuredDistance > 5 && measuredDistance < 60) {
    if (!AirplaneInGate) { // L'avion vient d'arriver
      AirplaneInGate = true; // Met à jour l'état pour "avion présent"
      Serial.println("Airplane Detected!");
      screen.show(0, 0, 255, "Welcome to", "Toulouse Airport");
      delay(3000);
    }
  } else { // L'avion est parti de la porte (distance hors de l'intervalle)
    if (AirplaneInGate) { // L'avion était présent, mais est parti
      AirplaneInGate = false; // Met à jour l'état pour "sans avion"
      ATC_Message = "";
      screen.show(255, 255, 255, ATC_Message, "");
      Serial.println("Airplane Left!");
    }
  }
}

void Wifi_Conected_Check() {
  // Vérifie la connexion Wi-Fi
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }
}