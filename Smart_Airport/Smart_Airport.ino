#include "Wifi_Access.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

WifiManager wifi(WiFi_ssid, WiFi_Password);

Led lamp(LightPin);
Led debugLight(LED_BUILTIN_AUX);
Buzzer alarmBuzzer(BuzzerPin);
MoteurToit moteur(MotorPin);
ScreenManager screen(D2,D1);

CapteurLuminosite lux(1 ,"Luminosité", LightSensorPin);
Button emergencyButton(2, "Emergence Button", PushButtonPin);
Button touchButton(3, "Lampe", TouchButtonPin);
TemperatureHumiditySensor weatherSensor(0x44);


// Variables pour le contrôle de l'angle
int angle = 0;       // Angle initial
int step = 10;       // Incrément de l'angle

void setup() {
  Serial.begin(9600); // Inicialises the Serial Monitor for debugin
  delay(100);

  wifi.init();              // Inicialises the Wi-Fi
  lamp.init();              // Inicialises the lamp
  alarmBuzzer.init();       // Inicialises the alarm
  screen.init();
  lux.init();               // Inicialises the light sensor
  moteur.init();            // Inicialises the motor
  emergencyButton.init();   // Inicialises the push button
  touchButton.init();       // Inicialises the touch button
  //weatherSensor.init();     // Inicialises the weather sensor
}

void loop() {

  bool AlarmActivated = false;
  if (emergencyButton.IsActivated() == true)
    AlarmActivated = true;
  while(AlarmActivated == true){
    screen.setrgb(255, 0, 0); // Configura a tela para vermelho
    alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
    //if(Cloud_DesableAlarm == "1")
    //  AlarmActivated = false;
  }
  screen.setrgb(255, 255, 255); // Configura a tela para branco
  
  // Function that reads the light sensor and sets the angle of the motor
  moteur.setAngle(lux.mesurer()/10);

  // Function that detects the ultrasonic sensor (closer then 100cm) and sends a message to the screen ("Welcome to the airport")
  if (distanceSensor.mesurer() > 10 && distanceSensor.mesurer() < 100)
    screen.show(0, 0, 255, "Welcome to", airportName + " Airport");
   

/*
  weatherSensor.show();

  screen.show(0,255,0,"Hello, Grove!","RGB Backlight!");
  screen.show(0,0,255,"ESP8266 Rocks!","I2C LCD Test!");
  screen.setrgb(0,0,255);

  alarmBuzzer.playFireAlarmPattern(200, 100, 1000); // Réglage des durées
*/


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
