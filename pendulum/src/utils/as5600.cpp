#include "src/utils/as5600.h"

#include <Wire.h>
#include <Arduino.h>

#include "src/utils/log_macros.h"

// static int as5600_offset = 0; // owned here, invisible everywhere else

int16_t as5600_read_raw() {
	Wire.beginTransmission(INNER_AS5600_ADDR);
	Wire.write(RAW_ANGLE_REG);
	Wire.endTransmission(false);

	Wire.requestFrom(INNER_AS5600_ADDR, 2);
	int16_t high = Wire.read();
	int16_t low = Wire.read();
	return ((high & 0x0F) << 8) | low;
}

// double as5600_read_rads(int16_t offset) {
// 	int16_t raw = as5600_read_raw();
// 	return (raw - offset) * (2.0 * PI / 4096.0) - PI;
// }

double as5600_read_rads(int16_t offset) {
	int16_t raw = as5600_read_raw();
	int adjusted = ((int)raw - (int)offset % 4096 + 4096) % 4096;
	return adjusted * (2.0 * PI / 4096.0);
}
