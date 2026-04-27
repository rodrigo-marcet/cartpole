#include "I2C_setup.h"

#include <Wire.h>

#include "src/utils/log_macros.h"

void init_i2c() {
	Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
	Wire.begin();
	Wire.setClock(I2C_FREQ);
	BOOT_LOG("I2C OK");
}
