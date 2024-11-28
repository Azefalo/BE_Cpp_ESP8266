#include "Wifi_Access.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

WifiManager wifi(WiFi_ssid, WiFi_Password);

Led lamp(LightPin);
MoteurToit moteur(MotorPin);
ScreenManager screen(D2,D1);

CapteurLuminosite lux(1 ,"Luminosité", LightSensorPin);
Button emergencyButton(2, "Emergence Button", PushButtonPin);
Button touchButton(3, "Lampe", TouchButtonPin);
TemperatureHumiditySensor weatherSensor(D8);


// Variables pour le contrôle de l'angle
int angle = 0;       // Angle initial
int step = 10;       // Incrément de l'angle

void setup() {
  Serial.begin(9600); // Inicialises the Serial Monitor for debugin
  delay(100);

  wifi.init();              // Inicialises the Wi-Fi
  lamp.init();              // Inicialises the lamp
  screen.init();
  lux.init();               // Inicialises the light sensor
  moteur.init();            // Inicialises the motor
  emergencyButton.init();   // Inicialises the push button
  touchButton.init();       // Inicialises the touch button
  weatherSensor.init();     // Inicialises the weather sensor
  
}

void loop() {
  // Tests the light activation
  lamp.on();
  delay(500);
  lamp.off();
  delay(500);

  // Mesures the light with the sensor and shows the amount of light detected
  lux.mesurer();
  lux.afficherValeur();

  if (emergencyButton.IsActivated() == true){
    lamp.on();
  }
  else{
    lamp.off();
  delay(300);
  }

  weatherSensor.show();

  screen.show(0,255,0,"Hello, Grove!","RGB Backlight!");
  screen.show(0,0,255,"ESP8266 Rocks!","I2C LCD Test!");
  screen.setrgb(0,0,255);

/*
  // Fait tourner le servo de 0 à 180 degrés
  for (angle = 0; angle <= 180; angle += step) {
    moteur.setAngle(angle);      // Définit l'angle du servo
    Serial.print("Angle : ");      // Retour via la communication série
    Serial.println(angle);
    delay(500);                    // Attend pour permettre au servo de bouger
  }

  // Fait tourner le servo de 180 à 0 degrés
  for (angle = 180; angle >= 0; angle -= step) {
    moteur.setAngle(angle);        // Définit l'angle du servo
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
