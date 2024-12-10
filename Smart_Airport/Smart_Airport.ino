#include "credentials.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

WifiManager wifi(WiFi_ssid, WiFi_Password);
MqttClient mqttClient(mqttServer, mqttPort, mqttUser, mqttPassword);
ScreenManager screen(D2, D1);

Led lamp(LightPin);
Led debugLight(LED_BUILTIN_AUX);
Buzzer alarmBuzzer(BuzzerPin);
RoofMotor motor(MotorPin);

LightSensor lux(1, "Luminosity", LightSensorPin);
Button emergencyButton(2, "Emergency Button", PushButtonPin);
Button touchButton(3, "Lamp", TouchButtonPin);
UltrasonicSensor distanceSensor(7, "Ultrasonic", DistanceSensorPin);
TemperatureHumiditySensor weatherSensor(0x44);

void setup() {
  Serial.begin(9600);
  Initialization();
}

void loop() {
  mqttClient.loop();
  FireAlarmCheck();
  WindowsAutomaticOpenClose();
  LightAutomaticOnOff();
  AirplaneInGateCheck();
  WifiConnectedCheck();
}
