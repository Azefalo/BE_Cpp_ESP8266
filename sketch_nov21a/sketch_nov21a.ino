#include <Wire.h>
#include "Adafruit_SHT31.h"

// Inicializa o sensor SHT31
Adafruit_SHT31 sht31 = Adafruit_SHT31();

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial
  Wire.begin();         // Inicializa o barramento I2C

  if (!sht31.begin(0x44)) { // Endereço I2C padrão do sensor
    Serial.println("Não foi possível encontrar o sensor SHT31");
    while (1); // Se o sensor não for encontrado, para a execução
  }
}

void loop() {
  // Lê a temperatura e a umidade do sensor
  float temperature = sht31.readTemperature();
  float humidity = sht31.readHumidity();

  if (!isnan(temperature) && !isnan(humidity)) { // Verifica se os valores são válidos
    Serial.print("Temperatura: ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Umidade: ");
    Serial.print(humidity);
    Serial.println(" %");
  } else {
    Serial.println("Falha ao ler os dados do sensor!");
  }

  delay(2000); // Aguarda 2 segundos antes de fazer a próxima leitura
}
