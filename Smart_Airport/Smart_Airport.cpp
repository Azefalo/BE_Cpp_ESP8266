#include "Smart_Airport.hpp"
#include "Smart_Airport.ino.globals.h"
#include "credentials.hpp"
#include "ESP8266_Pins.hpp"
#include <Servo.h>

# define PIN_ERROR 0
# define DISTANCE_TROP_CURTE -5
# define DISTANCE_TROP_LONG -2

rgb_lcd lcd; // Initialisation de l'écran Grove LCD
Servo servoMotor;

void Inicialization(){
  try{
    wifi.init();         // Inicialises the Wi-Fi
    weatherSensor.init();    // Inicialises the weather sensor
    distanceSensor.init();

  } catch (const std::runtime_error& e) {
    Serial.println(e.what());
  }
  lamp.init();         // Inicialises the lamp
  alarmBuzzer.init();  // Inicialises the alarm
  screen.init();
  lux.init();              // Inicialises the light sensor
  moteur.init();           // Inicialises the motor
  emergencyButton.init();  // Inicialises the push button
  touchButton.init();      // Inicialises the touch button
  // Connexion au serveur MQTT
  mqttClient.connectMQTT();
  mqttClient.subscribeData("alarmstop", messageCallback);
}

// Construtor
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

// Inicialises the Wi-Fi conection
void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnecting to Wi-Fi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > 10000) { // Timeout de 10 segundos
      throw std::runtime_error("Wi-Fi connection timed out.");
    }
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Verifica se está conectado ao Wi-Fi
bool WifiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// Retorna o endereço IP
String WifiManager::getIP() {
  if (isConnected()) {
    return WiFi.localIP().toString();
  } else {
    return "Not connected";
  }
}

// Reconects the Wi-Fi if the conection is lost
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


ScreenManager::ScreenManager(byte SDA,byte SCL):SDA(SDA),SCL(SCL){
  Wire.begin(SDA, SCL);
};
void ScreenManager::init(){
  lcd.begin(16, 2);
  ScreenManager::show (255,255,255,"Init","Wait");
  ScreenManager::setrgb (0, 255, 0);
}

void ScreenManager::show (uint8_t  r , uint8_t  g , uint8_t  b,String Message1="",String Message2=""){
  lcd.clear();
  ScreenManager::setrgb( r , g , b);
  lcd.setCursor(0, 0); // Ligne 0, Colonne 0
  lcd.print(Message1);
  lcd.setCursor(0, 1); // Ligne 0, Colonne 1
  lcd.print(Message2);
  delay(3000);
}

void ScreenManager::setrgb(uint8_t r , uint8_t g , uint8_t b){
  lcd.setRGB(r, g, b);
}


// Implémentation de MqttClient
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
            Serial.print("Échec, rc=");
            Serial.println(mqttClient.state());
            delay(5000);
        }
    }
}

void MqttClient::publishData(const char* topic, float data1,float data2,bool data3) {
    char payload[50];
    snprintf(payload, sizeof(payload), "%.2f/%.2f/%d", data1, data2, data3 ? 1 : 0);
    mqttClient.publish(topic, payload);
    Serial.print("Publié sur ");
    Serial.print(topic);
    Serial.print(" : ");
    Serial.println(payload);
}
// Abonnement à un sujet
void MqttClient::subscribeData(const char* topic, void (*callback)(char*, uint8_t*, unsigned int)) {
    mqttClient.setCallback(callback);
    mqttClient.subscribe(topic);

    Serial.print("Abonné au sujet : ");
    Serial.println(topic);
}
void MqttClient::loop() {
    mqttClient.loop();
}


bool AlarmActivated = false; 
void messageCallback(char* topic, uint8_t* payload, unsigned int length) {
    Serial.print("Message reçu sur le sujet : ");
    Serial.println(topic);

    // On lit le payload et on l'interprète
    String message = "";
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    Serial.print("Message : ");
    Serial.println(message);
    screen.show(255,255,0,message,"");
    // Exemple : si le message est "1", on active l'alarme
    if (message == "1") {
        AlarmActivated = false;
    }
}
void Fire_Alarm(){
  // Maintenir la connexion MQTT active et vérifier le payload
  mqttClient.loop(); 
  mqttClient.publishData("data",weatherSensor.getTemperature(),weatherSensor.getHumidity(),AlarmActivated);
  if (emergencyButton.IsActivated() == true)
    AlarmActivated = true;
  while(AlarmActivated == true){
    // Maintenir la connexion MQTT active et vérifier le payload
    mqttClient.loop(); 
    mqttClient.publishData("data",weatherSensor.getTemperature(),weatherSensor.getHumidity(),AlarmActivated);
    screen.setrgb(255, 0, 0); 
    alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
  }
  screen.setrgb(0, 255, 0);
  delay(2000);
}



// Construtor para inicializar com o endereço I2C
TemperatureHumiditySensor::TemperatureHumiditySensor(byte address = 0x44) : i2cAddress(address) {}

// Inicializa o sensor
void TemperatureHumiditySensor :: init(){
  if (!TemperatureHumiditySensor::begin()) {
    throw std::runtime_error("Échec de la communication avec le capteur SHT31.");
  }
}
bool TemperatureHumiditySensor :: begin() {
  return sht31.begin(i2cAddress);
}

// Retorna a temperatura atual em Celsius
float TemperatureHumiditySensor :: getTemperature() {
  return sht31.readTemperature();
}

// Retorna a umidade atual em porcentagem
float TemperatureHumiditySensor :: getHumidity() {
  return sht31.readHumidity();
}

void TemperatureHumiditySensor :: show(){
  if (TemperatureHumiditySensor::isValidReading()) {
    // Lê e exibe a temperatura
    float temperature = TemperatureHumiditySensor::getTemperature();
    Serial.print("Température: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Lê e exibe a umidade
    float humidity = TemperatureHumiditySensor::getHumidity();
    Serial.print("Humidité: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Erreur de lecture du capteur.");
  }
}

// Verifica se os valores são válidos
bool TemperatureHumiditySensor :: isValidReading() {
  return !isnan(getTemperature()) && !isnan(getHumidity());
}

// Constructeur pour initialiser la pin de l'actionneur
Actuator::Actuator(byte pin):pin(pin){}

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

// Méthode pour allumer la LED
void Led::on() {
  digitalWrite(getPin(), HIGH);  // Utilisation de getPin() pour récupérer la pin
}

// Méthode pour éteindre la LED
void Led::off() {
  digitalWrite(getPin(), LOW);  // Utilisation de getPin() pour récupérer la pin
}

// Constructeur de la classe Buzzer
Buzzer::Buzzer(byte pin) : Actuator(pin) {}

// Méthode pour jouer un motif d'alarme
void Buzzer::playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns) {
  for (int i = 0; i < 3; i++) {    // Trois bips courts
    digitalWrite(getPin(), HIGH);  // Utilise getPin() pour récupérer la broche
    delay(shortBeepDuration);
    digitalWrite(getPin(), LOW);  // Utilise getPin()
    delay(shortBeepInterval);
  }
  delay(pauseBetweenPatterns);  // Pause entre les motifs
}

// Méthode pour activer un son continu
void Buzzer::SetTone() {
    tone(getPin(), 1000); // Utilise getPin()
}

// Méthode pour arrêter le son
void Buzzer::SetnoTone() {
    noTone(getPin()); // Utilise getPin()
}

// Constructeur de la classe Led qui appelle le constructeur de la classe Actuator
MoteurToit::MoteurToit(byte pin) : Actuator(pin) {}

// Méthode d'initialisation de la LED
void MoteurToit::init(){
  servoMotor.attach(pin);
}

// Méthode pour allumer la LED
void MoteurToit::setAngle(int angle) {
  servoMotor.write(angle);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//définition de la classe Capteur
Capteur::Capteur(int id, String type, byte pin) : pin(pin), id(id), type(type) {}

void Capteur::init() {
  pinMode(pin, INPUT);
}

float Capteur :: mesurer(){
  return 0;
}
void Capteur :: afficherValeur(){}


//définition du capteur de luminosité
CapteurLuminosite :: CapteurLuminosite(int id, String type, byte pin): Capteur(id, type, pin), valeurLuminosite(0) {}

float CapteurLuminosite :: mesurer(){
  valeurLuminosite = analogRead(pin);
  return valeurLuminosite;
}

void CapteurLuminosite :: afficherValeur(){
  Serial.print("Luminosité : ");
  Serial.println(valeurLuminosite);
}
 
Button :: Button(int id, String type, byte pin) : Capteur(id, type, pin){}
  
bool Button :: IsActivated(){
  if (digitalRead(pin)==true){
    return true;
  }else{
    return false;
  }
}

UltrasonicSensor :: UltrasonicSensor(int id, String type, byte pin) : Capteur(id, type, pin){}

// Fonction pour mesurer la distance
int UltrasonicSensor :: measureDistance() {
  // Envoie une impulsion ultrasonique
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  
  // Lit la durée de l'écho
  pinMode(pin, INPUT);
  unsigned long startTime = micros();
  while (digitalRead(pin) == LOW) {
    if (micros() - startTime > 30000) { // Timeout de 30 ms
      return DISTANCE_TROP_CURTE; // Retourne -1 si aucun signal reçu
    }
  }

  unsigned long echoStart = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - echoStart > 30000) { // Timeout si l'écho est trop long
      return DISTANCE_TROP_LONG; // Retourne -1 pour signal trop long
    }
  }
  unsigned long echoEnd = micros();
  
  // Calcule la durée et la distance
  duration = echoEnd - echoStart;
  distance = duration * 0.034 / 2;

  return distance;
}
