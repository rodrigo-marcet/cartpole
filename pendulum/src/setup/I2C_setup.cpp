#include "src/setup/I2C_setup.h"

#include <Wire.h>

#include "src/utils/log_macros.h"

void init_i2c() {
	Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
	Wire.begin();
	Wire.setClock(I2C_FREQ);
	i2c_scan();
	BOOT_LOG("I2C OK");
}

void i2c_scan() {
	for (uint8_t addr = 1; addr < 127; addr++) {
		Wire.beginTransmission(addr);
		if (Wire.endTransmission() == 0) {
			BOOT_LOG("Found device at 0x%02X", addr);
		}
	}
}
