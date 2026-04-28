#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"

enum class OdriveCalibrationState : uint8_t {
	SAVE_INIT_POS,
	ENABLE_VELOCITY_CONTROL_POSITIVE,
	MOVE_TO_POSITIVE_LIMIT,
	SAVE_UPPER_LIMIT,
	MANAGE_ERRORS_1,
	WAIT_FOR_CLOSED_LOOP_1,
	ENABLE_POSITION_CONTROL_1,
	GO_TO_INIT_POS,
	ENABLE_VELOCITY_CONTROL_NEGATIVE,
	MOVE_TO_NEGATIVE_LIMIT,
	SAVE_LOWER_LIMIT,
	MANAGE_ERRORS_2,
	WAIT_FOR_CLOSED_LOOP_2,
	CALCULATE_VALUES,
	ENABLE_POSITION_CONTROL_2,
	GO_TO_MID_POINT,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus odrive_calibration(ODriveCalibrationResult *result);

bool move_to_limit(Direction direction);

bool move_to_position(float position);

struct RailLimits {
	bool ok = false;
	float midpoint = 0.0f;
	float upper_limit = 0.0f;
	float lower_limit = 0.0f;
};

RailLimits calculate_limits(float lower_limit, float upper_limit);
