#pragma once

#include <cstdint>

enum Color : uint32_t {
	RED = 0xFF0000,
	ORANGE = 0xFF5500,
	YELLOW = 0xFFAA00,
	GREEN = 0x00FF00,
	BLUE = 0x0000FF,
	PURPLE = 0x8000FF,
	MAGENTA = 0xFF00FF,
	WHITE = 0xFFFFFF,
	OFF = 0x000000
};

void initLED();
void setLED(uint32_t color);
void haltWithLED(uint32_t color);
