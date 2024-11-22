const int lightSensorPin = A0; // Pin analogique connecté au capteur
int sensorValue = 0;          // Variable pour stocker la lecture du capteur

void setup() {
  Serial.begin(9600); // Initialise la communication série
  pinMode(lightSensorPin, OUTPUT);
}

void loop() {
  // Lire la valeur analogique (0 à 1023)
  sensorValue = analogRead(lightSensorPin);

  // Convertir la valeur en intensité lumineuse (en fonction de vos besoins)
  // Par exemple, simplement afficher la valeur brute
  Serial.print("Luminosité: ");
  Serial.println(sensorValue);

  delay(500); // Petite pause pour éviter une lecture continue trop rapide
}