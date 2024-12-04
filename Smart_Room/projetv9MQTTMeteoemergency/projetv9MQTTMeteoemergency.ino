#include "PROJETV9MQTTMeteoemergency.hpp"
// Definition des pins de l' ESP8266

const int LightSensorPin    = A0;
const int MotorPin          = D3;
const int PushButtonPin     = D5;
const int TouchButtonPin    = D6;
const int DistanceSensorPin = D7;
const int LightPin          = D8;
const int BuzzerPin         = D9;
// Informations réseau
const char* ssid = "Wifizinho";
const char* password = "Senha123";

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
// Variable globale pour l'état de l'alarme
bool AlarmActivated = false; 
void messageCallback(char* topic, uint8_t* payload, unsigned int length) {
    Serial.print("Message reçu sur le sujet : ");
    Serial.println(topic);

//    Serial.print("Message : ");
//    for (unsigned int i = 0; i < length; i++) {
//        Serial.print((char)payload[i]);
//    }
//    Serial.println();

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

void setup() {
  Serial.begin(9600);  // Initialise la communication série
   // Initialisation de la connexion Wi-Fi
  wifiManager.init();
  alarmBuzzer.init();       // Inicialisation de l'alarme
  emergencyButton.init();   // Inicialisation du bouton poussoir 
  screen.init();
  // Connexion au serveur MQTT
  mqttClient.connectMQTT();
  mqttClient.subscribeData("alarmstop", messageCallback);
  sensor.init(); // Initialisation du capteur
}

void loop() {

  // Maintenir la connexion MQTT active et vérifier le payload
  mqttClient.loop(); 
  mqttClient.publishData("data",sensor.getTemperature(),sensor.getHumidity(),AlarmActivated);
  if (emergencyButton.IsActivated() == true){
    AlarmActivated = true;
  }
  while(AlarmActivated == true){
   // Maintenir la connexion MQTT active et vérifier le payload
    mqttClient.loop(); 
    mqttClient.publishData("data",sensor.getTemperature(),sensor.getHumidity(),AlarmActivated);
    screen.setrgb(255, 0, 0); 
    alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
    //if(Cloud_DesableAlarm == "1")
    //  AlarmActivated = false;
  }
  screen.setrgb(0, 255, 0);



  //mqttClient.publishData("data",sensor.getTemperature(),sensor.getHumidity());


  delay(2000);  // Attente de 2 secondes avant la prochaine lecture
}
