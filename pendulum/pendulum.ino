#include <Arduino.h>
#include <Wire.h>

#include "src/hfsm.h"

void setup(){
	Serial.begin(115200);
	delay(3000);
	Wire.begin(15, 16); // SDA=GPIO15, SCL=GPIO16
}

void loop(){
	hfsm();
}
