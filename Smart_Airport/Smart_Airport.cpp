#include "Smart_Airport.hpp"
#include "Smart_Airport.ino.globals.h"
#include "credentials.hpp"
#include "ESP8266_Pins.hpp"
#include <Servo.h>
extern String ATC_Message = "";

rgb_lcd lcd; // Initialization of the Grove LCD screen
Servo servoMotor;

std::vector<Sensor*> sensors = {&lux, &emergencyButton, &touchButton};
std::vector<Actuator*> actuators = {&lamp, &debugLight, &alarmBuzzer, &motor};

void Initialization() {
  screen.init();

  // Initialize all sensors
  for (auto sensor : sensors) {
    sensor->init();
  }
  // Initialize all actuators
  for (auto actuator : actuators) {
    actuator->init();
  }
  
  try {
    wifi.init();         // Initializes the Wi-Fi
    weatherSensor.init();    // Initializes the weather sensor
    distanceSensor.init();
    // Connect to the MQTT server
    mqttClient.connectMQTT();
  } catch (const std::runtime_error& e) {
    Serial.println(e.what());
  }
  mqttClient.subscribeData("alarmstop", messageCallback);
}

// Constructor
WifiManager::WifiManager(const char* ssid, const char* password) : ssid(ssid), password(password) {}

// Initializes the Wi-Fi connection
void WifiManager::init() {
  WiFi.begin(ssid, password);
  Serial.print("\n\nConnecting to Wi-Fi");
  unsigned long startAttemptTime = millis();

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startAttemptTime > 10000) { // 10 seconds timeout
      throw std::runtime_error("Wi-Fi connection timed out.");
    }
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

// Checks if connected to Wi-Fi
bool WifiManager::isConnected() {
  return WiFi.status() == WL_CONNECTED;
}

// Returns the IP address
String WifiManager::getIP() {
  if (isConnected()) {
    return WiFi.localIP().toString();
  } else {
    return "\nNot connected";
  }
}

// Reconnects the Wi-Fi if the connection is lost
void WifiManager::reconnect() {
  if (!isConnected()) {
    Serial.println("Reconnecting to Wi-Fi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nReconnected to Wi-Fi.");
  }
}

ScreenManager::ScreenManager(byte SDA, byte SCL) : SDA(SDA), SCL(SCL) {
  Wire.begin(SDA, SCL);
}

void ScreenManager::init() {
  lcd.begin(16, 2);
  ScreenManager::show(255, 128, 0, "Initializing ...", "Please Wait");
}

void ScreenManager::show(uint8_t r, uint8_t g, uint8_t b, String Message1, String Message2) {
  lcd.clear();
  ScreenManager::setrgb(r, g, b);
  lcd.setCursor(0, 0); // Line 0, Column 0
  lcd.print(Message1);
  lcd.setCursor(0, 1); // Line 0, Column 1
  lcd.print(Message2);
  delay(1000);
}

void ScreenManager::setrgb(uint8_t r, uint8_t g, uint8_t b) {
  lcd.setRGB(r, g, b);
}

ScreenManager& ScreenManager::operator<<(const String& message) {
  show(255, 255, 0, "     Update     ", message); // Display the message on line 1
  return *this;
}

// Implementation of MqttClient
MqttClient::MqttClient(const char* server, int port, const char* user, const char* password)
    : server(server), port(port), user(user), password(password), mqttClient(wifiClient) {
  mqttClient.setServer(server, port);
}

void MqttClient::connectMQTT() {
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT server...");
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str(), user, password)) {
      Serial.println("Connected to MQTT server!");
    } else {
      throw std::runtime_error("MQTT connection timed out.");
      delay(5000);
    }
  }
}

void MqttClient::publishData(const char* topic, float data1, float data2, bool data3) {
  char payload[50];
  snprintf(payload, sizeof(payload), "%.2f/%.2f/%d", data1, data2, data3 ? 1 : 0);
  mqttClient.publish(topic, payload);
  Serial.print("Published to ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(payload);
}

// Subscribe to a topic
void MqttClient::subscribeData(const char* topic, void (*callback)(char*, uint8_t*, unsigned int)) {
  mqttClient.setCallback(callback);
  mqttClient.subscribe(topic);

  Serial.print("Subscribed to topic: ");
  Serial.println(topic);
}

void MqttClient::loop() {
  mqttClient.loop();
}

bool AlarmActivated = false;

void messageCallback(char* topic, uint8_t* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  // Read the payload and interpret it
  String receivedMessage = "";
  for (unsigned int i = 0; i < length; i++) {
    receivedMessage += (char)payload[i];
  }
  
  Serial.print("Message: ");
  Serial.println(receivedMessage);
  
  // If the message is "1", deactivate the alarm
  if (receivedMessage == "1") {
    AlarmActivated = false;
  } else {
    ATC_Message = receivedMessage;
    screen << receivedMessage;
  }
}

void FireAlarmCheck() {
  // Keep the MQTT connection active and check the payload
  mqttClient.loop(); 
  mqttClient.publishData("data", weatherSensor.getTemperature(), weatherSensor.getHumidity(), AlarmActivated);
  if (emergencyButton.isActivated() == true) {
    AlarmActivated = true;
    while (AlarmActivated == true) {
      // Keep the MQTT connection active and check the payload
      mqttClient.loop(); 
      mqttClient.publishData("data", weatherSensor.getTemperature(), weatherSensor.getHumidity(), AlarmActivated);
      screen.show(255, 16, 0, "Attention! fire", "alarm activated");
      alarmBuzzer.playFireAlarmPattern(200, 100, 1000);
    }
    screen.show(0, 255, 0, "Fire alarm", "Deactivated");
    delay(1000);
  }
}

void WindowsAutomaticOpenClose() {
  // Function that reads the light sensor and sets the angle of the motor
  motor.setAngle(lux.measure() / 10);
}

bool LampActivated = false; // Global variable to track the lamp state

void LightAutomaticOnOff() {
  // Read the light sensor value
  int lightLevel = lux.measure();
  
  // Check the conditions to turn the lamp on or off
  if (lightLevel < 200 || touchButton.isActivated()) {
    if (!LampActivated) { // Only turn on if not already on
      LampActivated = true;
      lamp.on();
    }
  } else if (LampActivated && !touchButton.isActivated() && lightLevel > 200) {
    LampActivated = false; // Only turn off if it was on
    lamp.off();
  }
}

void updateDisplay() {
  String temperature = String(weatherSensor.getTemperature(), 1);
  String humidity = String(weatherSensor.getHumidity(), 1);

  // Build the second line of text
  String infoLine = "T:" + temperature + "oC H:" + humidity + "%";

  // Display the information on the screen
  screen.show(255, 255, 255, ATC_Message, infoLine);
}

bool AirplaneInGate = false;

void AirplaneInGateCheck() {
  // Read the distance from the sensor
  int measuredDistance = distanceSensor.measureDistance();
  updateDisplay();
  // Check if the airplane is at the gate (distance between 5 and 60 cm)
  if (measuredDistance > 5 && measuredDistance < 60) {
    if (!AirplaneInGate) { // Airplane just arrived
      AirplaneInGate = true; // Update the state to "airplane present"
      Serial.println("Airplane Detected!");
      screen.show(0, 0, 255, "Welcome to", "Toulouse Airport");
      delay(3000);
    }
  } else { // Airplane left the gate (distance out of range)
    if (AirplaneInGate) { // Airplane was present, but left
      AirplaneInGate = false; // Update the state to "no airplane"
      ATC_Message = "";
      screen.show(255, 255, 255, ATC_Message, "");
      Serial.println("Airplane Left!");
    }
  }
}

void WifiConnectedCheck() {
  // Verifies the Wi-Fi connection
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }
}

// Constructor to initialize with the I2C address
TemperatureHumiditySensor::TemperatureHumiditySensor(byte address) : i2cAddress(address) {}

// Initializes the sensor
void TemperatureHumiditySensor::init() {
  if (!TemperatureHumiditySensor::begin()) {
    throw std::runtime_error("Failed to communicate with the SHT31 sensor.");
  }
}

bool TemperatureHumiditySensor::begin() {
  return sht31.begin(i2cAddress);
}

// Returns the current temperature in Celsius
float TemperatureHumiditySensor::getTemperature() {
  return sht31.readTemperature();
}

// Returns the current humidity in percentage
float TemperatureHumiditySensor::getHumidity() {
  return sht31.readHumidity();
}

void TemperatureHumiditySensor::show() {
  if (TemperatureHumiditySensor::isValidReading()) {
    // Read and display the temperature
    float temperature = TemperatureHumiditySensor::getTemperature();
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // Read and display the humidity
    float humidity = TemperatureHumiditySensor::getHumidity();
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Sensor reading error.");
  }
}

// Checks if the values are valid
bool TemperatureHumiditySensor::isValidReading() {
  return !isnan(getTemperature()) && !isnan(getHumidity());
}

// Constructor to initialize the actuator pin
Actuator::Actuator(byte pin) : pin(pin) {}

// Method to initialize the pin in OUTPUT mode
void Actuator::init() {
  pinMode(pin, OUTPUT);
}

// Method to access the pin
byte Actuator::getPin() const {
  return pin;
}

// Constructor of the Led class that calls the constructor of the Actuator class
Led::Led(byte pin) : Actuator(pin) {}

// Method to turn on the LED
void Led::on() {
  digitalWrite(getPin(), HIGH);  // Use getPin() to get the pin
}

// Method to turn off the LED
void Led::off() {
  digitalWrite(getPin(), LOW);  // Use getPin() to get the pin
}

// Constructor of the Buzzer class
Buzzer::Buzzer(byte pin) : Actuator(pin) {}

// Method to play a fire alarm pattern
void Buzzer::playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns) {
  for (int i = 0; i < 3; i++) {    // Three short beeps
    digitalWrite(getPin(), HIGH);  // Use getPin() to get the pin
    delay(shortBeepDuration);
    digitalWrite(getPin(), LOW);  // Use getPin()
    delay(shortBeepInterval);
  }
  delay(pauseBetweenPatterns);  // Pause between patterns
}

// Method to activate a continuous sound
void Buzzer::setTone() {
  tone(getPin(), 1000); // Use getPin()
}

// Method to stop the sound
void Buzzer::setNoTone() {
  noTone(getPin()); // Use getPin()
}

// Constructor of the RoofMotor class that calls the constructor of the Actuator class
RoofMotor::RoofMotor(byte pin) : Actuator(pin) {}

// Method to initialize the motor
void RoofMotor::init() {
  servoMotor.attach(pin);
}

// Method to set the angle of the motor
void RoofMotor::setAngle(int angle) {
  servoMotor.write(angle);
}

// Definition of the Sensor class
Sensor::Sensor(int id, String type, byte pin) : pin(pin), id(id), type(type) {}

void Sensor::init() {
  pinMode(pin, INPUT);
}

float Sensor::measure() {
  return 0;
}

void Sensor::displayValue() {}

// Definition of the LightSensor class
LightSensor::LightSensor(int id, String type, byte pin) : Sensor(id, type, pin), lightValue(0) {}

float LightSensor::measure() {
  lightValue = analogRead(pin);
  return lightValue;
}

void LightSensor::displayValue() {
  Serial.print("Light: ");
  Serial.println(lightValue);
}

// Definition of the Button class
Button::Button(int id, String type, byte pin) : Sensor(id, type, pin) {}

bool Button::isActivated() {
  return digitalRead(pin) == HIGH;
}

// Definition of the UltrasonicSensor class
UltrasonicSensor::UltrasonicSensor(int id, String type, byte pin) : Sensor(id, type, pin) {}

// Function to measure the distance
int UltrasonicSensor::measureDistance() {
  // Send an ultrasonic pulse
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);
  
  // Read the echo duration
  pinMode(pin, INPUT);
  unsigned long startTime = micros();
  while (digitalRead(pin) == LOW) {
    if (micros() - startTime > 30000) { // 30 ms timeout
      return -1; // Return -1 if no signal received
    }
  }

  unsigned long echoStart = micros();
  while (digitalRead(pin) == HIGH) {
    if (micros() - echoStart > 30000) { // Timeout if the echo is too long
      return -1; // Return -1 for too long signal
    }
  }
  unsigned long echoEnd = micros();
  
  // Calculate the duration and distance
  duration = echoEnd - echoStart;
  distance = duration * 0.034 / 2;

  return distance;
}
