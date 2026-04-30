#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class KillswitchLimitsState : uint8_t {
	CHECK_CONTROL_LOOP_CONTROL,
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	CHECK_VELOCITY,
	ENABLE_TORQUE_CONTROL,
	BREAK,
	ENABLE_POSITION_CONTROL,
	GO_TO_CLOSEST_LIMIT,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus killswitch_limits_sequence(const EncoderEstimatesResult &fb, const ODriveCalibrationResult &limits);
