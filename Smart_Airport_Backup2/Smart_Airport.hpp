#ifndef SMART_AIRPORT_HPP
#define SMART_AIRPORT_HPP

#include <Arduino.h>          // Arduino Core Library
#include <ESP8266WiFi.h>      // Library for ESP8266 Wi-Fi Module
#include <Wire.h>             // Library for I2C communication
#include "rgb_lcd.h"          // Library for Grove LCD RGB Backlight
#include "Adafruit_SHT31.h"   // Library for Adafruit SHT31 Temperature and Humidity Sensor
#include <PubSubClient.h>     // MQTT Library
#include <stdexcept>          // Library for exceptions

void Initialization(); 
void messageCallback(char* topic, uint8_t* payload, unsigned int length);
void Fire_Alarm_Check();
void Windows_Automatic_Open_Close();
void Light_Automatic_On_Off();
void Airplane_In_Gate_Check();
void Wifi_Connected_Check();

class WifiManager {
private:
  const char* ssid;       // Wi-Fi's name
  const char* password;   // Wi-Fi's password
public:
  WifiManager(const char* ssid, const char* password);
  void init();
  bool isConnected();
  String getIP();
  void reconnect();
};

class ScreenManager {
private:
  byte SDA;
  byte SCL;
public:
  ScreenManager(byte SDA, byte SCL);
  void init();
  void show(uint8_t r, uint8_t g, uint8_t b, String Message1 = "", String Message2 = "");
  void setrgb(uint8_t r, uint8_t g, uint8_t b);
};

class MqttClient {
private:
  const char* server;
  int port;
  const char* user;
  const char* password;
  WiFiClient wifiClient;
  PubSubClient mqttClient;
public:
  MqttClient(const char* server, int port, const char* user, const char* password);
  void connectMQTT();
  void publishData(const char* topic, float data1, float data2, bool data3);
  void subscribeData(const char* topic, void (*callback)(char*, uint8_t*, unsigned int));
  void loop();
};

class TemperatureHumiditySensor {
private:
  byte i2cAddress;
  Adafruit_SHT31 sht31;
public:
  TemperatureHumiditySensor(byte address = 0x44);
  void init();
  bool begin();
  float getTemperature();
  float getHumidity();
  void show();
  bool isValidReading();
};

class Actuator {
protected:
  byte pin;
public:
  Actuator(byte pin);
  void init();
  byte getPin() const;
};

class Led : public Actuator {
public:
  Led(byte pin);
  void on();
  void off();
};

class Buzzer : public Actuator {
public:
  Buzzer(byte pin);
  void playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns);
  void SetTone();
  void SetnoTone();
};

class MoteurToit : public Actuator {
private:
  Servo servoMotor;
public:
  MoteurToit(byte pin);
  void init();
  void setAngle(int angle);
};

class Sensor {
protected:
  byte pin;
  int id;
  String type;
public:
  Sensor(int id, String type, byte pin);
  virtual void init();
  virtual float measure();
  virtual void displayValue();
};

class LightSensor : public Sensor {
private:
  float lightValue;
public:
  LightSensor(int id, String type, byte pin);
  float measure();
  void displayValue();
};

class Button : public Sensor {
public:
  Button(int id, String type, byte pin);
  bool IsActivated();
};

class UltrasonicSensor : public Sensor {
private:
  unsigned long duration;
  float distance;
public:
  UltrasonicSensor(int id, String type, byte pin);
  int measureDistance();
};

#endif // SMART_AIRPORT_HPP
