#pragma once

#include <cstdint>

#define INNER_AS5600_ADDR 0x36
#define RAW_ANGLE_REG 0x0C

int16_t as5600_read_raw();

double as5600_read_rads(double offset);
