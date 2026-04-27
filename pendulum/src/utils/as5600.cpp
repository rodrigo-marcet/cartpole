#include "src/utils/as5600.h"

#include <Wire.h>
#include <Arduino.h>

static int as5600_offset = 0; // owned here, invisible everywhere else

int as5600_read_raw() {
	Wire.beginTransmission(AS5600_ADDR);
	Wire.write(RAW_ANGLE_REG);
	Wire.endTransmission(false);
	Wire.requestFrom(AS5600_ADDR, 2);
	int high = Wire.read();
	int low = Wire.read();
	return ((high & 0x0F) << 8) | low;
}

void set_as5600_offset(int o) {
	as5600_offset = o;
}
int get_as5600_offset() {
	return as5600_offset;
}

double as5600_read_rads() {
	int raw = as5600_read_raw();
	return (raw - as5600_offset) * (2.0 * PI / 4096.0) - PI;
}
