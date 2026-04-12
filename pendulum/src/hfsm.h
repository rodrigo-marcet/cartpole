#pragma once

#include <Arduino.h>

enum class HFSMState : uint8_t{
	Calibration = 1,
	Running = 2
};

void hfsm();