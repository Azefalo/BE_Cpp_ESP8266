#include "projetv8MQTT.hpp"

// Informations réseau
const char* ssid = "iPhone de PDS";
const char* password = "chatraule10";

// Informations MQTT
const char* mqttServer = "h1e3430a.ala.dedicated.aws.emqxcloud.com";
const int mqttPort = 1883;
const char* mqttUser = "pierrele123";
const char* mqttPassword = "Ledragon123@";

// Objets Wi-Fi et MQTT
WifiManager wifiManager(ssid, password);
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword);

void setup() {
    Serial.begin(115200);

    // Initialisation de la connexion Wi-Fi
    wifiManager.init();

    // Connexion au serveur MQTT
    mqttClient.connectMQTT();
}

void loop() {
    // Maintenir la connexion MQTT active
    mqttClient.loop();

    // Exemple de publication des données
    float temperature = 25.5;  // Exemple de valeur statique
    mqttClient.publishData("data", temperature);

    delay(5000); // Publication toutes les 5 secondes
}
