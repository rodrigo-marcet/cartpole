#pragma once

#include <Arduino.h>

enum class SequenceStatus : uint8_t { RUNNING, DONE = 254, ERROR = 255 };

struct ODriveCalibrationResult {
	float midpoint = 0;
	float upper_limit = 0;
	float lower_limit = 0;
	float physical_upper_limit = 0;
	float physical_lower_limit = 0;
};

struct CalibrationResult {
	ODriveCalibrationResult odrive_result;
};
