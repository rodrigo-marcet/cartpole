#pragma once

#define AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

// reads raw angle from sensor, returns 12-bit value
int as5600_read_raw();

// offset management
void set_as5600_offset(int offset);
int get_as5600_offset();

// derived
double as5600_read_rads();
