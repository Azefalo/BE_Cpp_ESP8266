#include"projetv7.hpp"
#define BUZZER_PIN D7 // Définit la broche du buzzer
Buzzer alarmBuzzer(BUZZER_PIN);

void setup() {
    // Initialisation
    Serial.begin(9600);
    Serial.println("GBUGBYBBGFAQAFFFZAQFAE");
    alarmBuzzer.init();
    Serial.println("Système d'Alarme Incendie Démarré");
}

void loop() {
    alarmBuzzer.SetTone();
    delay(5000);
    alarmBuzzer.SetnoTone();
    // Exécute le motif de l'alarme incendie
    alarmBuzzer.playFireAlarmPattern(200, 100, 1000); // Réglage des durées

}
