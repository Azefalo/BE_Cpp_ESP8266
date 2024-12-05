#include"projetv5.hpp"


ScreenManager screen(D2,D1);

void setup() {
screen.init();
screen.setrgb(0,255,0);
}

void loop() {

  screen.show(0,255,0,"Hello, Grove!","RGB Backlight!");
  screen.show(0,0,255,"ESP8266 Rocks!","I2C LCD Test!");
  screen.setrgb(0,0,255);
}
