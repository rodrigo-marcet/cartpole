#include "I2C_setup.h"

#include <Wire.h>

void initI2C() {
	Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
	Wire.begin();
	Wire.setClock(I2C_FREQ);
	Serial.println("[BOOT] I2C OK");
}
