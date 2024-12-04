#include "credentials.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

WifiManager wifi("Wifizinho", "Senha123");
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword);
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
}

void loop() {
  screen.show (255,255,255,"","");
/*
  Fire_Alarm();
    
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

*/
  // Function that detects the ultrasonic sensor (closer then 100cm) and sends a message to the screen ("Welcome to the airport")
  Serial.print("Distance mesured:");
  Serial.println(distanceSensor.measureDistance());
  delay(500);
  
  
  // This is to garantee that this mensagem is going to show only once the airplane has arrived
  bool AirplaneInGate = false;
  if (distanceSensor.measureDistance() > 5 && distanceSensor.measureDistance() < 60){
    AirplaneInGate = true;
  }else if (distanceSensor.measureDistance() == 0) {
    AirplaneInGate = false;
  }
  if (AirplaneInGate) {
    Serial.println("Airplane Detected!");
    screen.show(0, 0, 255, "Welcome to", "Toulouse Airport");
    delay(2000);
  }
  



  // Verifies the Wi-Fi conection
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }

  //Airport_TLS.Light();
  //Airport_TLS.Window();
}
