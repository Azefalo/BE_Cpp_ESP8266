#include"projetv7.hpp"
//#include "Wifi_Access.hpp"
#include <PubSubClient.h> // Bibliothèque MQTT
#include <Wire.h>
#include "rgb_lcd.h" // Bibliothèque dédiée au Grove LCD RGB Backlight
#include <Servo.h>
rgb_lcd lcd; // Initialisation de l'écran Grove LCD
Servo servoMotor;


// Constructeur
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

// Initialise la connexion Wi-Fi
void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnexion au Wi-Fi en cours");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnexion au Wi-Fi établie.");
  Serial.print("Adresse IP : ");
  Serial.println(WiFi.localIP());
}

// Vérifie si le module est connecté au Wi-Fi
bool WifiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// Retourne l'adresse IP
String WifiManager::getIP() {
  if (isConnected()) {
    return WiFi.localIP().toString();
  } else {
    return "Non connecté";
  }
}

// Rétablit la connexion Wi-Fi si elle est perdue
void WifiManager::reconnect() {
  if (!isConnected()) {
    Serial.println("Reconnexion au Wi-Fi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nConnexion au Wi-Fi rétablie.");
  }
}



ScreenManager::ScreenManager(byte SDA,byte SCL):SDA(SDA),SCL(SCL){
Wire.begin(SDA, SCL);
};
void ScreenManager::init(){
lcd.begin(16, 2);
ScreenManager::show (255,255,255,"Initializing...","Please wait!");
ScreenManager::setrgb (0, 255, 0);
}


void ScreenManager::show (int r , int g , int b,String Message1="",String Message2=""){
  lcd.clear();
  ScreenManager::setrgb( r , g , b);
  lcd.setCursor(0, 0); // Ligne 0, Colonne 0
  lcd.print(Message1);
  lcd.setCursor(0, 1); // Ligne 0, Colonne 1
  lcd.print(Message2);
  delay(3000);
}

void ScreenManager::setrgb(int r , int g , int b){
  lcd.setRGB(r, g, b);
}




// Constructeur de la classe Actuator
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

//// Initialisation du buzzer
//void Buzzer::init() {
//    Actuator::init(); // Appelle la méthode init() de la classe Actuator
//}

// Méthode pour jouer un motif d'alarme
void Buzzer::playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns) {
    for (int i = 0; i < 3; i++) { // Trois bips courts
        digitalWrite(getPin(), HIGH); // Utilise getPin() pour récupérer la broche
        delay(shortBeepDuration);
        digitalWrite(getPin(), LOW); // Utilise getPin()
        delay(shortBeepInterval);
    }
    delay(pauseBetweenPatterns); // Pause entre les motifs
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

UltrasonicSensor :: UltrasonicSensor(int id, String type, byte pin) : Capteur(id, type, pin){}

// Fonction pour mesurer la distance
int UltrasonicSensor :: measureDistance() {
  // Envoie une impulsion ultrasonique
  pinMode(pin, OUTPUT);
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
      return -1; // Retourne -1 si aucun signal reçu
    }
  }

  unsigned long echoStart = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - echoStart > 30000) { // Timeout si l'écho est trop long
      return -1; // Retourne -1 pour signal trop long
    }
  }
  unsigned long echoEnd = micros();
  
  // Calcule la durée et la distance
  duration = echoEnd - echoStart;
  distance = duration * 0.034 / 2;

  return distance;
}



MqttClient::MqttClient(const char* ssid, const char* password, const char* mqttServer, const int mqttPort, const char* mqttUser, const char* mqttPassword)
    : ssid(ssid), password(password), mqttServer(mqttServer), mqttPort(mqttPort), mqttUser(mqttUser), mqttPassword(mqttPassword), client(espClient) {}

//void MqttClient::connectWiFi() {
//    Serial.print("Connexion au réseau Wi-Fi: ");
//    Serial.print(ssid);
//
//    WiFi.begin(ssid, password);
//
//    // Attendre la connexion
//    while (WiFi.status() != WL_CONNECTED) {
//        delay(1000);
//        Serial.print(".");
//    }
//
//    Serial.println();
//    Serial.print("Connecté au Wi-Fi avec l'adresse IP: ");
//    Serial.println(WiFi.localIP());
//}

void MqttClient::connectMQTT() {
    Serial.print("Connexion au serveur MQTT: ");
    Serial.println(mqttServer);

    // Tentative de connexion au serveur MQTT
    while (!client.connected()) {
        Serial.print("Tentative de connexion au serveur MQTT...");
        
        // Tentative de connexion avec utilisateur et mot de passe
        if (client.connect("ESP8266Client", mqttUser, mqttPassword)) {
            Serial.println("Connecté au serveur MQTT");
        } else {
            Serial.print("Échec de la connexion, code d'erreur: ");
            Serial.print(client.state());
            delay(5000); // Attente avant de réessayer
        }
    }
}

void MqttClient::publishData(const char* topic, float data) {
    char payload[50];
    snprintf(payload, sizeof(payload), "%.2f", data);  // Convertir la donnée en chaîne avec 2 décimales
    if (client.publish(topic, payload)) {
        Serial.print("Données publiées sur le sujet: ");
        Serial.print(topic);
        Serial.print(" Valeur: ");
        Serial.println(payload);
    } else {
        Serial.println("Échec de la publication des données.");
    }
}

void MqttClient::loop() {
    client.loop();  // Maintenir la connexion MQTT active
}

 
