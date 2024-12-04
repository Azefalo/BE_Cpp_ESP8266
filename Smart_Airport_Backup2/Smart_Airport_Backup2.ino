#include "credentials.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

// Wi-Fi and MQTT setup
WifiManager wifi(WiFi_ssid, WiFi_Password);
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword);

// Screen setup
ScreenManager screen(D2, D1);

// Actuators
Led lamp(LightPin);
Led debugLight(LED_BUILTIN_AUX);
Buzzer alarmBuzzer(BuzzerPin);
RoofMotor motor(MotorPin);

// Sensors
LightSensor lux(1, "Luminosity", LightSensorPin);
Button emergencyButton(2, "Emergency Button", PushButtonPin);
Button touchButton(3, "Lamp", TouchButtonPin);
UltrasonicSensor distanceSensor(7, "Ultrasonic", DistanceSensorPin);
TemperatureHumiditySensor weatherSensor(0x44);

void setup() {
  Serial.begin(9600); // Initialize the Serial Monitor for debugging
  delay(100);
  
  Initialization();
  
  screen.show(0, 255, 0, "Initialized with", "success!");
  delay(1000);
}

void loop() {
  screen.show(255, 255, 255, "", "");

  Airplane_In_Gate_Check();
  Fire_Alarm_Check();
  Windows_Automatic_Open_Close();
  Light_Automatic_On_Off();
 
  Wifi_Connected_Check();
  
  //Airport_TLS.Light();
  //Airport_TLS.Window();
}
