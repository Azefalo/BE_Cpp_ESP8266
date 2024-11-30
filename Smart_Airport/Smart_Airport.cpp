#include "Smart_Airport.hpp"
#include "Wifi_Access.hpp"
#include <Servo.h>

rgb_lcd lcd; // Initialisation de l'écran Grove LCD
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




// Construtor para inicializar com o endereço I2C
TemperatureHumiditySensor::TemperatureHumiditySensor(byte address = 0x44) : i2cAddress(address) {}

// Inicializa o sensor
void TemperatureHumiditySensor :: init(){
  if (!TemperatureHumiditySensor::begin()) {
    Serial.println("Échec de la communication avec le capteur SHT31.");
    while (1) delay(1);  // Fica preso aqui em caso de erro
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
 
Button :: Button(int id, String type, byte pin) : Capteur(id, type, pin){}
  
bool Button :: IsActivated(){
  if (digitalRead(pin)==true){
    return true;
  }else{
    return false;
  }
}   
