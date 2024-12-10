#include "ESP8266_Pins.hpp" // Définit les broches utilisées par les capteurs et actionneurs
#include "Smart_Airport.hpp" // Contient les définitions des classes et fonctions du projet
#include "credentials.hpp" // Contient les identifiants Wi-Fi et MQTT (SSID et mot de passe)

// Gestionnaires de réseau et communication
WifiManager wifi(WiFi_ssid, WiFi_Password); // Gestionnaire Wi-Fi pour connecter l'ESP8266
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword); // Client MQTT pour la communication serveur

// Gestionnaire d'écran
ScreenManager screen(D2, D1); // Gestionnaire d'écran LCD RGB connecté aux broches I2C (D2 = SDA, D1 = SCL)

// Actionneurs
Led lamp(LightPin); // LED principale simulant la lumière
Led debugLight(LED_BUILTIN_AUX); // LED intégrée utilisée pour le débogage
Buzzer alarmBuzzer(BuzzerPin); // Buzzer utilisé pour alerter en cas d'incendie
MoteurToit moteur(MotorPin); // Servo moteur pour gérer l'ouverture/fermeture des fenêtres

// Capteurs
CapteurLuminosite lux(1, "Luminosity", LightSensorPin); // Capteur de luminosité connecté à une broche analogique
Button emergencyButton(2, "Emergency Button", PushButtonPin); // Bouton d'urgence pour les alertes
Button touchButton(3, "Lampe", TouchButtonPin); // Bouton tactile pour activer/désactiver manuellement la lampe
UltrasonicSensor distanceSensor(7, "Ultrasonic", DistanceSensorPin); // Capteur ultrasonique pour détecter la distance (par exemple, avion)
TemperatureHumiditySensor weatherSensor(0x44); // Capteur de température et d'humidité (adresse I2C par défaut 0x44)

// Fonction d'initialisation du système
void setup() {
  Serial.begin(9600); // Initialise la communication série à 9600 bauds pour le débogage
  delay(100); // Petite pause pour stabiliser les composants

  Inicialization(); // Appelle la fonction globale d'initialisation (capteurs, actionneurs, Wi-Fi, MQTT, etc.)

  // Affiche un message de confirmation d'initialisation réussie sur l'écran LCD
  screen.show(0, 255, 0, "Inicialized with", "success!");
  delay(1000); // Pause pour permettre la lecture du message avant de passer à autre chose
}

// Boucle principale du programme
void loop() {
  mqttClient.loop(); // Maintient la connexion au serveur MQTT active et traite les messages entrants

  // Effectue des vérifications et actions automatiques
  Airplane_In_Gate_Check(); // Vérifie la présence d'un avion à la porte en utilisant le capteur ultrasonique
  Fire_Alarm_Check(); // Vérifie et gère l'état de l'alarme incendie (activation/désactivation)
  Windows_Automatic_Open_Close(); // Ajuste automatiquement les fenêtres en fonction de la luminosité
  Light_Automatic_On_Off(); // Allume ou éteint automatiquement la lampe selon les conditions (luminosité ou bouton tactile)

  Wifi_Conected_Check(); // Vérifie si le Wi-Fi est toujours connecté et tente une reconnexion si nécessaire

}
