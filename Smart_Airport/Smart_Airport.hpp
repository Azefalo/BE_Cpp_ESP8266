#ifndef SMART_AIRPORT_HPP
#define SMART_AIRPORT_HPP

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include "Adafruit_SHT31.h"
#include <PubSubClient.h>
#include <stdexcept>
#include <vector>

// Global functions
void updateDisplay();
void Initialization();
void messageCallback(char* topic, uint8_t* payload, unsigned int length);
void FireAlarmCheck();
void WindowsAutomaticOpenClose();
void LightAutomaticOnOff();
void AirplaneInGateCheck();
void WifiConnectedCheck();

class WifiManager {
private:
  const char* ssid;
  const char* password;

public:
  WifiManager(const char* ssid, const char* password);
  void init();
  bool isConnected();
  String getIP();
  void reconnect();
};

class TemperatureHumiditySensor {
private:
    Adafruit_SHT31 sht31;
    byte i2cAddress;

public:
  TemperatureHumiditySensor(byte address);
  void init();
  bool begin();
  float getTemperature();
  float getHumidity();
  void show();
  bool isValidReading();
};

class ScreenManager {
private:
  byte SDA, SCL;
  int r, g, b;
  String Message1, Message2;

public:
  ScreenManager(byte SDA, byte SCL);
  void setrgb(uint8_t r, uint8_t g, uint8_t b);
  void show(uint8_t r, uint8_t g, uint8_t b, String Message1, String Message2);
  void init();
  ScreenManager& operator<<(const String& message);
};

class MqttClient {
private:
    const char* server;
    const int port;
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

class Actuator {
protected:
  byte pin;

public:
  Actuator(byte pin);
  virtual void init();
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
  void setTone();
  void setNoTone();
};

class RoofMotor : public Actuator {
private:
  int angle;

public:
  RoofMotor(byte pin);
  void init() override;
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
  int lightValue;

public:
  LightSensor(int id, String type, byte pin);
  float measure();
  void displayValue();
};

class Button : public Sensor {
private:
  bool activated;

public:
  Button(int id, String type, byte pin);
  bool isActivated();
};

class UltrasonicSensor : public Sensor {
private:
  long duration;
  int distance;

public:
  UltrasonicSensor(int id, String type, byte pin);
  int measureDistance();
};

extern WifiManager wifi;
extern MqttClient mqttClient;
extern ScreenManager screen;
extern Led lamp;
extern Led debugLight;
extern Buzzer alarmBuzzer;
extern RoofMotor motor;
extern LightSensor lux;
extern Button emergencyButton;
extern Button touchButton;
extern UltrasonicSensor distanceSensor;
extern TemperatureHumiditySensor weatherSensor;

#endif // SMART_AIRPORT_HPP
