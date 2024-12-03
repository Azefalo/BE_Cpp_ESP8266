#include "PROJETV9MQTTMeteoemergency.hpp"

// Definition des pins of the ESP8266

const int LightSensorPin    = A0;
const int MotorPin          = D3;
const int PushButtonPin     = D5;
const int TouchButtonPin    = D6;
const int DistanceSensorPin = D7;
const int LightPin          = D8;
const int BuzzerPin         = D9;
// Informations réseau
const char* ssid = "iPhone de PDS";
const char* password = "chatraule10";

// Informations MQTT
const char* mqttServer = "h1e3430a.ala.dedicated.aws.emqxcloud.com";
const int mqttPort = 1883;
const char* mqttUser = "pierrele123";
const char* mqttPassword = "projetairport";

// Création d'un objet pour le capteur de température et d'humidité
TemperatureHumiditySensor sensor(0x44);
Buzzer alarmBuzzer(BuzzerPin);
Button emergencyButton(2, "Emergence Button", PushButtonPin);                              
ScreenManager screen(D2,D1);
// Objets Wi-Fi et MQTT
WifiManager wifiManager(ssid, password);
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword);

//void setup() {
//    Serial.begin(115200);
//
//    // Initialisation de la connexion Wi-Fi
//    wifiManager.init();
//
//    // Connexion au serveur
//    mqttClient.connectMQTT();
//}
//
//void loop() {
//    // Maintenir la connexion MQTT active
//    mqttClient.loop();
//
//    // Exemple de publication des données
//    float temperature = 25.5;  // Exemple de valeur statique
//    mqttClient.publishData("data", temperature);
//
//    delay(5000); // Publication toutes les 5 secondes
//}


void setup() {
  Serial.begin(9600);  // Initialise la communication série
   // Initialisation de la connexion Wi-Fi
  wifiManager.init();
  alarmBuzzer.init();       // Inicialises the alarm
  emergencyButton.init();   // Inicialises the push button
  screen.init();
    // Connexion au serveur MQTT
  mqttClient.connectMQTT();

  sensor.init(); // Initialisation du capteur
}

void loop() {
   // Maintenir la connexion MQTT active
  mqttClient.loop();  
    // Vérifie si la lecture du capteur est valide et affiche les données
     // Exemple de publication des données
//    float temperature = 25.5;  // Exemple de valeur statique
  bool AlarmActivated = false;
  if (emergencyButton.IsActivated() == true) {
    AlarmActivated = true;
    screen.setrgb(255, 0, 0); // Configura a tela para vermelho
    for(int i=0; i<10; i++){
      alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
    }
  } else {
    screen.setrgb(255, 255, 255); // Configura a tela para branco
    delay(300);
  }



  mqttClient.publishData("data",sensor.getTemperature(),sensor.getHumidity());


  delay(2000);  // Attente de 2 secondes avant la prochaine lecture
}
