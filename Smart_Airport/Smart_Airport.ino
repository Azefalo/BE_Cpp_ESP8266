#include "Wifi_Access.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

// This function is a copy of the lines 8 to 19 to test if everithing
// is well defined and there is no error in the pins definition
void Pins_Exception_Test();

WifiManager wifi("Wifizinho", "Senha123");
ScreenManager screen(D2, D1);
// Actuators
Led lamp(LightPin);
Led debugLight(LED_BUILTIN_AUX);
Buzzer alarmBuzzer(BuzzerPin);
MoteurToit moteur(MotorPin);
// Sensors
CapteurLuminosite lux(1, "Luminosity", LightSensorPin);
Button emergencyButton(2, "Emergency Button", PushButtonPin);
Button touchButton(3, "Lampe", TouchButtonPin);
UltrasonicSensor distanceSensor(7, "Ultrasonic", DistanceSensorPin);
TemperatureHumiditySensor weatherSensor(0x44);

void setup() {
  Serial.begin(9600); // Inicialises the Serial Monitor for debugin
  delay(100);
  
  Inicialization();

  screen.show(0, 255, 0, "Inicialized with", "success!");
  delay(1000);
  screen.setrgb(255, 255, 255);
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
  
  // Turns on or off the airport lights acording to the inside light
  bool LampActivated = false;
  if (lux.mesurer() < 200 || touchButton.IsActivated()) {
    LampActivated = true;
    lamp.on();
    delay(500);
  } else if ((LampActivated == true && touchButton.IsActivated()) || lux.mesurer() > 200 ) {
    LampActivated = false;
    lamp.off();
    delay(500);
  }

  // Function that detects the ultrasonic sensor (closer then 100cm) and sends a message to the screen ("Welcome to the airport")
  Serial.println("Distance mesured:");
  Serial.println(distanceSensor.mesurer());
  delay(500);
  // This is to garantee that this mensagem is going to show only once the airplane has arrived
  bool AirplaneInGate = false;
  if (distanceSensor.mesurer() > 5 && distanceSensor.mesurer() < 60){
    AirplaneInGate = true;
  }else if (distanceSensor.mesurer() == 0) {
    AirplaneInGate = false;
  }
  if (AirplaneInGate) {
    Serial.println("Airplane Detected!");
    screen.show(0, 0, 255, "Welcome to", "Toulouse Airport");
  }
  

  if (AirplaneInGate) {
    // Mensage to the screen
  }


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
