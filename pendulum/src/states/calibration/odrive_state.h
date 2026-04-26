#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"

enum class OdriveCalibrationState : uint8_t {
	SAVE_INIT_POS = 0,
	ENABLE_VELOCITY_CONTROL_POSITIVE = 1,
	MOVE_TO_POSITIVE_LIMIT = 2,
	SAVE_UPPER_LIMIT = 3,
	ENABLE_POSITION_CONTROL_1 = 4,
	GO_TO_INIT_POS = 5,
	ENABLE_VELOCITY_CONTROL_NEGATIVE = 6,
	MOVE_TO_NEGATIVE_LIMIT = 7,
	SAVE_LOWER_LIMIT = 8,
	CALCULATE_VALUES = 9,
	ENABLE_POSITION_CONTROL_2 = 10,
	GO_TO_MID_POINT = 11,

	DONE = 254,
	ERROR = 255,
};

SequenceState odrive_calibration();

bool move_to_limit(Direction direction);

bool move_to_position(float position);

struct RailLimits {
	bool ok = false;
	float midpoint = 0.0f;
	float upper_limit = 0.0f;
	float lower_limit = 0.0f;
};

RailLimits calculate_limits(float lower_limit, float upper_limit);
