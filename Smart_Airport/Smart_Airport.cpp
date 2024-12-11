#include "Smart_Airport.hpp"
#include "Smart_Airport.ino.globals.h"
#include "credentials.hpp"
#include "ESP8266_Pins.hpp"
#include <Servo.h>
extern String ATC_Message = "";

rgb_lcd lcd; //Instantiation de l'écran Grove LCD
Servo servoMotor;

std::vector<Capteur*> sensors = {&lux, &emergencyButton, &touchButton}; 
std::vector<Actuator*> actuators = {&lamp, &debugLight, &alarmBuzzer, &moteur};


void Inicialization() {
  screen.init();

  // Initialize all sensors
  for (auto sensor : sensors) {
    sensor->init();
  }
  // Initialize all actuators
  for (auto actuator : actuators) {
    actuator->init();
  }
  
  try{
    wifi.init();         // teste Initialisation du Wifi
    weatherSensor.init();    // Initialisation du Capteur weather sensor
    distanceSensor.init();
    // Connexion au serveur MQTT
    mqttClient.connectMQTT(); //teste la connection MQTT
  } catch (const std::runtime_error& e) {
    Serial.println(e.what());             //affiche le type d'érreur
  }
  mqttClient.subscribeData("alarmstop", messageCallback); //publie le payload (message) sur le topic alarmstop du serveur MQTT
}

// Constructeur de WifiManage
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

// Initialisation de la connexion Wi-Fi
void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnecting to Wi-Fi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > 10000) { // Timeout au bout de 10 secondes si le Wi-Fi ne parvient pas à s'initialiser
      throw std::runtime_error("Wi-Fi connection timed out."); //lance une exception
    }
  }
  Serial.println("\nWi-Fi connected."); // sinon le Wi-Fi est connecté
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// vérifie si le Wi-Fi est connecté
bool WifiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// retourne une adresse IP si la connexion est établie
String WifiManager::getIP() {
  if (isConnected()) {
    return WiFi.localIP().toString(); 
  } else {
    return "\nNot connected";
  }
}

// Reconnecte le Wi-Fi si la connexion est perdue
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

void Wifi_Conected_Check(){
  // vérifies si la connexion est  perdue
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }
}
//Constructeur de ScreenManager (pour gérer l'affichage LCD)
ScreenManager::ScreenManager(byte SDA,byte SCL):SDA(SDA),SCL(SCL){
  Wire.begin(SDA, SCL);
};
//initialisation de l'écran LCD
void ScreenManager::init(){
  lcd.begin(16, 2);
  ScreenManager::show (255,128,0,"Initialising ...","Please Wait");
}

//Affichage de 2 messages sur l'écran LCD
void ScreenManager::show (uint8_t  r , uint8_t  g , uint8_t  b,String Message1="",String Message2=""){
  lcd.clear();
  ScreenManager::setrgb( r , g , b);
  lcd.setCursor(0, 0); // le 1er message est sur la Ligne 0, Colonne 0
  lcd.print(Message1);
  lcd.setCursor(0, 1); //  le 2ième message est sur la Ligne 0, Colonne 1
  lcd.print(Message2);
  delay(1000);
}

// définis la couleur de l'écran LCD
void ScreenManager::setrgb(uint8_t r , uint8_t g , uint8_t b){
  lcd.setRGB(r, g, b);
}

ScreenManager& ScreenManager::operator<<(const String& message) {
    show(255, 255, 0, "     Update     ", message); //  redéfinition de l'opérateur affichage en cas de réception d'un message via Node-red 
    return *this;
}

// Constructeur du protocole de communication  Mqtt
MqttClient::MqttClient(const char* server, int port, const char* user, const char* password)
    : server(server), port(port), user(user), password(password), mqttClient(wifiClient) {
    mqttClient.setServer(server, port);
}

//méthode pour connecter l'ESP8266 au serveur MQTT
void MqttClient::connectMQTT() {
    while (!mqttClient.connected()) {
        Serial.println("Connexion au serveur MQTT...");
        String clientId = "ESP8266Client-"; // définition du ClientID
        clientId += String(random(0xffff), HEX);
        if (mqttClient.connect(clientId.c_str(), user, password)) {
            Serial.println("Connecté au serveur MQTT !");  //connextion établie
        } else {
            throw std::runtime_error("Mqtt connection timed out."); // lancement d'une exception dans la méthode en cas de problème de connexion
            delay(5000);
        }
    }
}

// permet la publication des 3 payloads sur le serveur MQTT (température/humidité/signalisation de l'alarme activé ou désactivé en définissant le Topic
void MqttClient::publishData(const char* topic, float data1,float data2,bool data3) {
    char payload[50];
    snprintf(payload, sizeof(payload), "%.2f/%.2f/%d", data1, data2, data3 ? 1 : 0);
    mqttClient.publish(topic, payload);
    Serial.print("Publié sur ");
    Serial.print(topic);  
    Serial.print(" : ");
    Serial.println(payload);
}

// Abonnement à un sujet(topic) pour pouvoir écouter l'arrivée de payloads
void MqttClient::subscribeData(const char* topic, void (*callback)(char*, uint8_t*, unsigned int)) {
    mqttClient.setCallback(callback);
    mqttClient.subscribe(topic); //souscription à un topic 

    Serial.print("Abonné au sujet : ");
    Serial.println(topic);
}
void MqttClient::loop() {
    mqttClient.loop(); // Maintient la connexion au serveur MQTT active et traite les messages entrants
}


bool AlarmActivated = false; 
void messageCallback(char* topic, uint8_t* payload, unsigned int length) { //fonction utile pour récupérer le message du serveur et le traiter 
    Serial.print("Message reçu sur le sujet : ");
    Serial.println(topic);
    // On lit le payload et on l'interprète
    String receivedMessage = "";
    for (unsigned int i = 0; i < length; i++) {
        receivedMessage += (char)payload[i]; // reconstruction du message reçu 
    }
    
    Serial.print("Message : ");
    Serial.println(receivedMessage); //affichage du message reçu sur le serveur 
    
    // Si le message est "1", on désactive l'alarme
    if (receivedMessage == "1") {
        AlarmActivated = false;
    } else{
      ATC_Message=receivedMessage; //message reçu du Air traffic control system
      screen << receivedMessage; // affiche le message reçu sur l'écran LCD
    }
}
void Fire_Alarm_Check(){
  // Maintenir la connexion MQTT active et vérifier le payload
  mqttClient.loop(); 
  // publication des données de température,d'humidité et l'état de l'alarme (activé ou désactivé)
  mqttClient.publishData("data",weatherSensor.getTemperature(),weatherSensor.getHumidity(),AlarmActivated); 
  if (emergencyButton.IsActivated() == true){ // si l'alarme est activé
    AlarmActivated = true;
    while(AlarmActivated == true){
      // Maintenir la connexion MQTT active et vérifier le payload
      mqttClient.loop(); 
      mqttClient.publishData("data",weatherSensor.getTemperature(),weatherSensor.getHumidity(),AlarmActivated);
      screen.show (255,16,0,"Atention! fire","alarm activated");
      alarmBuzzer.playFireAlarmPattern(200, 100, 1000); //le buzzer sonne
    }
    screen.show (0,255,0,"Fire alarm","Desactivated"); //affichage de l'alarme désactivé sur l'écran LCD
    delay(1000);
  }
}

// une fonction qui lit la luminosité du milieu et définis l'angle de rotation du moteur pour fermer ou ouvrir le volet
void Windows_Automatic_Open_Close() {
  moteur.setAngle(lux.mesurer()/10);
}

bool LampActivated = false; // variable globale permettant de définir l'état de la lampe
void Light_Automatic_On_Off() { // // Fonction pour activer ou désactiver la lampe automatiquement en fonction de la luminosité ou d'un bouton tactile
  // Mesure le niveau de luminosité à l'aide d'un capteur (lux.mesurer())
  int lightLevel = lux.mesurer();
  
    // Vérifie les conditions pour activer ou désactiver la lampe
  if (lightLevel < 200 || touchButton.IsActivated()) {
    if (!LampActivated) {  // Allume la lampe uniquement si elle est actuellement éteinte
      LampActivated = true; // Met à jour l'état global de la lampe
      lamp.on(); 
    }
  } else if (LampActivated && !touchButton.IsActivated() && lightLevel > 200) {
    LampActivated = false; // Met à jour l'état global de la lampe à "éteinte"
    lamp.off(); 
  }
}

//foncfion pour afficher les messages sur l'écran LCD
void updateDisplay() {
    String temperature = String(weatherSensor.getTemperature(),1);
    String humidity = String(weatherSensor.getHumidity(),1);

    // Construire la deuxième ligne de texte
    String infoLine = "T:" + temperature + "oC H:" + humidity + "%";

    // Afficher les informations reçu du Air traffic control system sur l'écran
    screen.show(255, 255, 255, ATC_Message, infoLine);
}

bool AirplaneInGate = false;
void Airplane_In_Gate_Check(){
  // lire la distance entre la porte d'embarcation et l'avion 
  int measuredDistance = distanceSensor.measureDistance();
  updateDisplay();
  // vérifie si l'avion proche de la porte d'embarcation (distância entre 5 e 60 cm)
  if (measuredDistance > 5 && measuredDistance < 60) {
    if (!AirplaneInGate) { // si l'avion vient d'arriver
      AirplaneInGate = true; // mise à jour de la variable comme l'avion est près de la porte
      Serial.println("Airplane Detected!");
      screen.show(0, 0, 255, "Welcome to", "Toulouse Airport");
      delay(3000);
    }
  } else { // si l'avion n'est pas proche de la porte
    if (AirplaneInGate) { // si l'avion s'éloigne du gate pour décoller
      AirplaneInGate = false; 
      ATC_Message = "";
      screen.show(255, 255, 255, ATC_Message, ""); // avion parti
      Serial.println("Airplane Left!");
    }
  }
}




// Constructeur pour initialiser le capteur avec une adresse I2C (par défaut 0x44)
TemperatureHumiditySensor::TemperatureHumiditySensor(byte address = 0x44) : i2cAddress(address) {}

// Initialise le capteur
void TemperatureHumiditySensor :: init(){
  // Vérifie si le capteur est accessible, sinon lève une exception
  if (!TemperatureHumiditySensor::begin()) {
    throw std::runtime_error("Échec de la communication avec le capteur SHT31.");
  }
}

// Démarre la communication avec le capteur via l'adresse I2C
bool TemperatureHumiditySensor :: begin() {
  return sht31.begin(i2cAddress);
}

// Retourne la température actuelle mesurée par le capteur en degrés Celsius
float TemperatureHumiditySensor :: getTemperature() {
  return sht31.readTemperature();
}

// Retourne le taux d'humidité actuel mesuré par le capteur en pourcentage
float TemperatureHumiditySensor :: getHumidity() {
  return sht31.readHumidity();
}

// Affiche les valeurs mesurées (température et humidité) si elles sont valides
void TemperatureHumiditySensor :: show(){
  if (TemperatureHumiditySensor::isValidReading()) {
    // Lit et affiche la température
    float temperature = TemperatureHumiditySensor::getTemperature();
    Serial.print("Température: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Lit et affiche l'humidité
    float humidity = TemperatureHumiditySensor::getHumidity();
    Serial.print("Humidité: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    // Affiche un message d'erreur en cas de lecture invalide
    Serial.println("Erreur de lecture du capteur.");
  }
}

// Vérifie si les lectures de température et d'humidité sont valides (pas NaN)
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
  if (digitalRead(pin)==true){ //si le bouton activé
    return true;
  }else{
    return false;
  }
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
