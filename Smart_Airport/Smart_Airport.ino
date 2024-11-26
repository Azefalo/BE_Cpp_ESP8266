#include "Wifi_Access.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

WifiManager wifi(WiFi_ssid, WiFi_Password);

Led led(LightPin);
MoteurToit Moteur(MotorPin);

CapteurLuminosite lux(1 ,"Luminosité", LightSensorPin);


// Variables pour le contrôle de l'angle
int angle = 0;       // Angle initial
int step = 10;       // Incrément de l'angle

void setup() {
  Serial.begin(9600); // Inicialises the Serial Monitor for debugin
  delay(100);

  wifi.init();      // Inicialises the Wi-Fi
  led.init();       // Inicialises the lamp
  lux.init();       // Inicialises the light sensor
  Moteur.init();    // Inicialises the motor
  
}

void loop() {
  // Tests the light activation
  led.on();
  delay(500);
  led.off();
  delay(500);

  // Mesures the light with the sensor and shows the amount of light detected
  lux.mesurer();
  lux.afficherValeur();

/*
  // Fait tourner le servo de 0 à 180 degrés
  for (angle = 0; angle <= 180; angle += step) {
    Moteur.setAngle(angle);      // Définit l'angle du servo
    Serial.print("Angle : ");      // Retour via la communication série
    Serial.println(angle);
    delay(500);                    // Attend pour permettre au servo de bouger
  }

  // Fait tourner le servo de 180 à 0 degrés
  for (angle = 180; angle >= 0; angle -= step) {
    Moteur.setAngle(angle);        // Définit l'angle du servo
    Serial.print("Angle : ");      // Retour via la communication série
    Serial.println(angle);
    delay(500);                    // Attend pour permettre au servo de bouger
  }
*/


  // Verifies the Wi-Fi conection
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }

  //Airport_TLS.Light();
  //Airport_TLS.Window();
}
