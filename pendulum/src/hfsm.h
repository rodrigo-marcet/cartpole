#pragma once

#include <Arduino.h>

enum class HFSMState : uint8_t { CALIBRATION = 1, RUNNING = 2 };

void hfsm();
