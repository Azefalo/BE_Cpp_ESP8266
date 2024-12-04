#include "credentials.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

WifiManager wifi(WiFi_ssid, WiFi_Password);
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

  Airplane_In_Gate_Check();
  Fire_Alarm_Check();
  Windows_Automatic_Open_Close();
  Light_Automatic_On_Off();
 
  Wifi_Conected_Check();
  
  //Airport_TLS.Light();
  //Airport_TLS.Window();
}
