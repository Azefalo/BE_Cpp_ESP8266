#include <ESP8266WiFi.h>

// Substitua com suas credenciais de rede
const char* ssid = "Vai Brasil";
const char* password = "Senha123";

void setup() {
  Serial.begin(9600);
  Serial.println("Initializing Wi-Fi...");
  
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi connected.");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  // Nada aqui
}
