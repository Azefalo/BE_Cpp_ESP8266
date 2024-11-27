#include "projetv6.hpp"
#include <Wire.h>
#include "Adafruit_SHT31.h"


// Criação de um objeto para o sensor
TemperatureHumiditySensor sensor;

void setup() {
  Serial.begin(9600);  // Inicializa a comunicação serial
  Serial.println("Test du Grove - Capteur de température et d'humidité (SHT31)");
}

void loop() {
  // Verifica se a leitura é válida
  

  delay(2000);  // Espera 2 segundos antes da próxima leitura
}
