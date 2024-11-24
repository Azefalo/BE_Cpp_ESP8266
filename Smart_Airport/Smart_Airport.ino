#include "Wifi_Access.hpp"
#include "Smart_Airport.hpp"
#include "ESP8266_Pins.hpp"

Led led(LightLampPin);
CapteurLuminosite lux(1 ,"Luminosité", LightSensorPin);
WifiManager wifi(WiFi_ssid, WiFi_Password); 


void setup() {
  wifi.init();      // Inicialises the Wi-Fi
  led.init();       // Inicialises the lamp
  lux.init();       // Inicialises the light sensor
  
  Serial.begin(9600); // Inicialises the Serial Monitor for debugin 
}

void loop() {
  // Tests the light activation
  led.on();
  delay(500);
  led.off();
  delay(500);

  // Mesures the light with the sensor and shows the amount of light detected
  lux.mesurer();
  lux.afficherValeur();

  // Verifies the Wi-Fi conection
  if (!wifi.isConnected()) {
    wifi.reconnect();
  }

  //Airport_TLS.Light();
  //Airport_TLS.Window();
}
