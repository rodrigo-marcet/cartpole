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
struct AS5600CalibrationResult {
	int raw_offset = 0;
};

struct CalibrationResult {
	ODriveCalibrationResult odrive_result;
	AS5600CalibrationResult inner_encoder_result;
};
