#include <Arduino.h>

// Classe Buzzer
class Buzzer {
private:
    int pin;

public:
    Buzzer(int buzzerPin) : pin(buzzerPin) {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, LOW); // Buzzer inicia desligado
    }

    // Método para tocar um padrão de bipes
    void playFireAlarmPattern(int shortBeepDuration, int shortBeepInterval, int pauseBetweenPatterns) {
        for (int i = 0; i < 3; i++) { // Três bipes curtos
            digitalWrite(pin, HIGH);
            delay(shortBeepDuration);
            digitalWrite(pin, LOW);
            delay(shortBeepInterval);
        }
        delay(pauseBetweenPatterns); // Pausa entre os padrões
    }
};

#define BUZZER_PIN D7 // Define o pino do buzzer
Buzzer alarmBuzzer(BUZZER_PIN);

void setup() {
    // Inicialização
    Serial.begin(9600);
    Serial.println("Sistema de Alarme de Incêndio Iniciado");
    pinMode(LED_BUILTIN_AUX, INPUT);
}

void loop() {
    // Executa o padrão do alarme de incêndio
    alarmBuzzer.playFireAlarmPattern(200, 100, 1000); // Ajuste dos tempos
    digitalWrite(LED_BUILTIN_AUX, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN_AUX, LOW);
    delay(500);
}
