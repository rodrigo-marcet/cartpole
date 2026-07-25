#include "src/setup/I2C_setup.h"

#include "src/setup/LED_setup.h"

#include "src/utils/log_macros.h"

void init_i2c() {
	Wire.setPins(INNER_I2C_SDA_PIN, INNER_I2C_SCL_PIN);
	Wire.begin();
	Wire.setClock(I2C_FREQ);

	if (!i2c_scan(Wire)) {
		BOOT_ERROR("I2C failed find any valid adress on the inner encoder");
		halt_with_led(Color::BLUE);
	}

	Wire1.setPins(OUTER_I2C_SDA_PIN, OUTER_I2C_SCL_PIN);
	Wire1.begin();
	Wire1.setClock(I2C_FREQ);

	if (!i2c_scan(Wire1)) {
		BOOT_ERROR("I2C failed find any valid adress on the outer encoder");
		halt_with_led(Color::BLUE);
	}

	BOOT_LOG("I2C OK");
}

bool i2c_scan(TwoWire &wire) {
	uint8_t count = 0;
	for (uint8_t addr = 1; addr < 127; addr++) {
		wire.beginTransmission(addr);
		if (wire.endTransmission() == 0) {
			count += 1;
			BOOT_LOG("[I2C] Found device at 0x%02X", addr);
		}
	}
	return count > 0;
}
