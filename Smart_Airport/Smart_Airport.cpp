#include "Smart_Airport.hpp"
#include "Wifi_Access.hpp"

Servo servoMotor;


// Construtor
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

// Inicialises the Wi-Fi conection
void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
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
  if (digitalRead(pin)==false){
    return true;
  }else{
    return false;
  }
}   