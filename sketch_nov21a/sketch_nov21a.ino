#define LED_PIN 13

Led led(LED_PIN);

void setup() {
  Led.init(); 
}

void loop() {
  Led.on();
  delay(500);
  Led.off();
  delay(500);
  
}
