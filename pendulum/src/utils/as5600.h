#pragma once

#include <cstdint>

#define AS5600_ADDR 0x36
#define INNER_AS5600_IDX 0
#define OUTER_AS5600_IDX 1
#define RAW_ANGLE_REG 0x0C

int16_t as5600_read_raw(const uint16_t idx = INNER_AS5600_IDX);

float as5600_read_rads(float offset, const uint16_t idx = INNER_AS5600_IDX);
