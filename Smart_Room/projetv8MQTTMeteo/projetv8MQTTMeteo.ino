#include "projetv8MQTTMeteo.hpp"

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
// Objets Wi-Fi et MQTT
WifiManager wifiManager(ssid, password);
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword);

//void setup() {
//    Serial.begin(115200);
//
//    // Initialisation de la connexion Wi-Fi
//    wifiManager.init();
//
//    // Connexion au serveur MQTT
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
  mqttClient.publishData("data",sensor.getTemperature(),sensor.getHumidity());


  delay(2000);  // Attente de 2 secondes avant la prochaine lecture
}
