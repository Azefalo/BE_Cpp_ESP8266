#include <Wire.h>
#include "Adafruit_SHT31.h"

// Criação de um objeto para o sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial
  Serial.println("Test du Grove - Capteur de température et d'humidité (SHT31)");

  // Inicializa o sensor
  if (!sht31.begin(0x44)) { // Endereço padrão do SHT31
    Serial.println("Échec de la communication avec le capteur SHT31.");
    while (1) delay(1); // Fica preso aqui em caso de erro
  }
}

void loop() {
  // Lê os valores de temperatura e umidade
  float temperature = sht31.readTemperature(); // Temperatura em °C
  float humidity = sht31.readHumidity();       // Umidade em %

  // Verifica se os valores são válidos
  if (!isnan(temperature) && !isnan(humidity)) {
    Serial.print("Température: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidité: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Erreur de lecture du capteur.");
  }

  delay(2000); // Espera 2 segundos antes da próxima leitura
}
