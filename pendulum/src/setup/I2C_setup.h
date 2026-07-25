#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "src/config.h"

void init_i2c();

bool i2c_scan(TwoWire &wire);
