#include "projetv6.hpp"



// Criação de um objeto para o sensor
TemperatureHumiditySensor sensor(D3);

void setup() {
  Serial.begin(9600);  // Inicializa a comunicação serial
  Serial.println("Test du Grove - Capteur de température et d'humidité (SHT31)");
  sensor.init();
}

void loop() {
  // Verifica se a leitura é válida
  sensor.show();

  delay(2000);  // Espera 2 segundos antes da próxima leitura
}
