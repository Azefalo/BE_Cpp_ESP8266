#include "Smart_Airport.hpp"
#include "Smart_Airport.ino.globals.h"
#include "credentials.hpp"
//#include "ESP8266_Pins.hpp"
#include <Servo.h>

// Variables globales
extern String ATC_Message = "";
rgb_lcd lcd; // Initialisation de l'écran Grove LCD
Servo servoMotor; // Initialisation du servo moteur pour les fenêtres
bool AlarmActivated = false;
bool LampActivated = false;
bool AirplaneInGate = false;

// Vecteurs de capteurs et actionneurs
std::vector<Sensor*> sensors = {&luxSensor, &emergencyButton, &touchButton};
std::vector<Actuator*> actuators = {&lamp, &debugLight, &alarmBuzzer, &roofMotor};

// Fonction d'initialisation du système
void Initialization() {
  screen.init();

  // Initialiser tous les capteurs
  for (auto sensor : sensors) {
    sensor->init();
  }
  // Initialiser tous les actionneurs
  for (auto actuator : actuators) {
    actuator->init();
  }
  
  try {
    wifi.init(); // Initialiser le Wi-Fi
    weatherSensor.init(); // Initialiser le capteur de température et d'humidité
    distanceSensor.init(); // Initialiser le capteur ultrasonique
    mqttClient.connectMQTT(); // Se connecter au serveur MQTT
  } catch (const std::runtime_error& e) {
    Serial.println(e.what());
  }
  mqttClient.subscribeData("alarmstop", messageCallback);
}

// Classe WifiManager
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnexion au Wi-Fi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > 10000) { // Timeout de 10 secondes
      throw std::runtime_error("Le délai de connexion Wi-Fi a expiré.");
    }
  }
  Serial.println("\nWi-Fi connecté.");
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
}

bool WifiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

String WifiManager::getIP() {
  if (isConnected()) {
    return WiFi.localIP().toString();
  } else {
    return "\nNon connecté";
  }
}

void WifiManager::reconnect() {
  init();
}

// Classe ScreenManager
ScreenManager::ScreenManager(byte SDA, byte SCL) : SDA(SDA), SCL(SCL) {
  Wire.begin(SDA, SCL);
}

void ScreenManager::init() {
  lcd.begin(16, 2);
  show(255, 128, 0, "Initialisation ...", "Veuillez patienter");
}

void ScreenManager::show(uint8_t r, uint8_t g, uint8_t b, String Message1, String Message2) {
  lcd.clear();
  setrgb(r, g, b);
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
  show(255, 255, 0, "     Mise à jour     ", message); // Affiche le message sur la ligne 1
  return *this;
}

// Classe MqttClient
MqttClient::MqttClient(const char* server, int port, const char* user, const char* password)
    : server(server), port(port), user(user), password(password), mqttClient(wifiClient) {}

void MqttClient::connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Connexion à MQTT...");
    if (mqttClient.connect("ESP8266Client", user, password)) {
      Serial.println("connecté");
    } else {
      Serial.print("échec avec l'état ");
      Serial.print(mqttClient.state());
      delay(2000);
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

// Callback pour les messages MQTT reçus
void messageCallback(char* topic, uint8_t* payload, unsigned int length) {
  Serial.print("Message reçu sur le sujet : ");
  Serial.println(topic);
  String receivedMessage = "";
  for (unsigned int i = 0; i < length; i++) {
    receivedMessage += (char)payload[i];
  }
  Serial.print("Message : ");
  Serial.println(receivedMessage);
  if (receivedMessage == "1") {
    AlarmActivated = false;
  } else {
    ATC_Message = receivedMessage;
    screen << receivedMessage;
  }
}

// Fonctions de vérification et de contrôle
void Fire_Alarm_Check() {
  mqttClient.loop();
  mqttClient.publishData("data", weatherSensor.getTemperature(), weatherSensor.getHumidity(), AlarmActivated);
  if (emergencyButton.IsActivated() == true) {
    AlarmActivated = true;
    while (AlarmActivated == true) {
      mqttClient.loop();
      mqttClient.publishData("data", weatherSensor.getTemperature(), weatherSensor.getHumidity(), AlarmActivated);
      screen.show(255, 16, 0, "Attention! feu", "alarme activée");
      alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
    }
    screen.show(0, 255, 0, "Alarme incendie", "Désactivée");
    delay(1000);
  }
}

void Windows_Automatic_Open_Close() {
  roofMotor.setAngle(luxSensor.measure() / 10);
}

void Light_Automatic_On_Off() {
  int lightLevel = luxSensor.measure();
  if (lightLevel < 200 || touchButton.IsActivated()) {
    if (!LampActivated) {
      LampActivated = true;
      lamp.on();
    }
  } else if (LampActivated && !touchButton.IsActivated() && lightLevel > 200) {
    LampActivated = false;
    lamp.off();
  }
}

void Airplane_In_Gate_Check() {
  int measuredDistance = distanceSensor.measureDistance();
  updateDisplay();
  if (measuredDistance > 5 && measuredDistance < 60) {
    if (!AirplaneInGate) {
      AirplaneInGate = true;
      Serial.println("Avion détecté !");
      screen.show(0, 0, 255, "Bienvenue à", "l'aéroport de Toulouse");
      delay(3000);
    }
  } else {
    if (AirplaneInGate) {
      AirplaneInGate = false;
      ATC_Message = "";
      screen.show(255, 255, 255, ATC_Message, "");
      Serial.println("Avion parti !");
    }
  }
}

void Wifi_Connected_Check() {
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }
}

// Classe TemperatureHumiditySensor
TemperatureHumiditySensor::TemperatureHumiditySensor(byte address) : i2cAddress(address) {}

void TemperatureHumiditySensor::init() {
  if (!begin()) {
    throw std::runtime_error("Échec de la communication avec le capteur SHT31.");
  }
}

bool TemperatureHumiditySensor::begin() {
  return sht31.begin(i2cAddress);
}

float TemperatureHumiditySensor::getTemperature() {
  return sht31.readTemperature();
}

float TemperatureHumiditySensor::getHumidity() {
  return sht31.readHumidity();
}

void TemperatureHumiditySensor::show() {
  if (isValidReading()) {
    float temperature = getTemperature();
    Serial.print("Température : ");
    Serial.print(temperature);
    Serial.println(" °C");

    float humidity = getHumidity();
    Serial.print("Humidité : ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Erreur de lecture du capteur.");
  }
}

bool TemperatureHumiditySensor::isValidReading() {
  return !isnan(getTemperature()) && !isnan(getHumidity());
}

// Classe Actuator
Actuator::Actuator(byte pin) : pin(pin) {}

void Actuator::init() {
  pinMode(pin, OUTPUT);
}

byte Actuator::getPin() const {
  return pin;
}

// Classe Led
Led::Led(byte pin) : Actuator(pin) {}

void Led::on() {
  digitalWrite(getPin(), HIGH);
}

void Led::off() {
  digitalWrite(getPin(), LOW);
}

// Classe Buzzer
Buzzer::Buzzer(byte pin) : Actuator(pin) {}

void Buzzer::playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(getPin(), HIGH);
    delay(shortBeepDuration);
    digitalWrite(getPin(), LOW);
    delay(shortBeepInterval);
  }
  delay(pauseBetweenPatterns);
}

void Buzzer::SetTone() {
  tone(getPin(), 1000);
}

void Buzzer::SetnoTone() {
  noTone(getPin());
}

// Classe RoofMotor
RoofMotor::RoofMotor(byte pin) : Actuator(pin) {}

void RoofMotor::init() {
  servoMotor.attach(pin);
}

void RoofMotor::setAngle(int angle) {
  servoMotor.write(angle);
}

// Classe Sensor
Sensor::Sensor(int id, String type, byte pin) : pin(pin), id(id), type(type) {}

void Sensor::init() {
  pinMode(pin, INPUT);
}

float Sensor::measure() {
  return 0;
}

void Sensor::displayValue() {}

// Classe LightSensor
LightSensor::LightSensor(int id, String type, byte pin) : Sensor(id, type, pin), lightValue(0) {}

float LightSensor::measure() {
  lightValue = analogRead(pin);
  return lightValue;
}

void LightSensor::displayValue() {
  Serial.print("Luminosité : ");
  Serial.println(lightValue);
}

// Classe Button
Button::Button(int id, String type, byte pin) : Sensor(id, type, pin) {}

bool Button::IsActivated() {
  return digitalRead(pin);
}

// Classe UltrasonicSensor
UltrasonicSensor::UltrasonicSensor(int id, String type, byte pin) : Sensor(id, type, pin) {}

int UltrasonicSensor::measureDistance() {
  // Implémentation de la mesure de distance
  return 0;
}

// Fonction pour mettre à jour l'affichage
void updateDisplay() {
  String temperature = String(weatherSensor.getTemperature(), 1);
  String humidity = String(weatherSensor.getHumidity(), 1);

  // Construire la deuxième ligne de texte
  String infoLine = "T:" + temperature + "oC H:" + humidity + "%";

  // Afficher les informations sur l'écran
  screen.show(255, 255, 255, ATC_Message, infoLine);
}
