#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class RunningSetupState : uint8_t {
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	ENABLE_POSITION_CONTROL,
	GO_TO_MID_POINT,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus setup_sequence(const EncoderEstimatesResult &fb, const ODriveCalibrationResult &limits);
