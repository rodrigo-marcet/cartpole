#include "src/utils/as5600.h"

#include <Wire.h>
#include <Arduino.h>

#include "src/utils/log_macros.h"

int16_t as5600_read_raw() {
	Wire.beginTransmission(INNER_AS5600_ADDR);
	Wire.write(RAW_ANGLE_REG);
	Wire.endTransmission(false);

	Wire.requestFrom(INNER_AS5600_ADDR, 2);
	int16_t high = Wire.read();
	int16_t low = Wire.read();
	return ((high & 0x0F) << 8) | low;
}

float as5600_read_rads(float offset) {
	int16_t raw = as5600_read_raw();

	float adjusted = std::fmod((float)raw - offset, 4096.0);
	if (adjusted < 0)
		adjusted += 4096.0;

	return adjusted * (2.0 * PI / 4096.0);
}
