#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class MainSequenceState : uint8_t {
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	ENABLE_TYPE_CONTROL,
	SINUSOIDAL_POS,
	POSITION_PID,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus main_sequence(MainSequenceState &current_state, const CalibrationResult &calibration_result,
                             const EncoderEstimatesResult &fb);

SequenceStatus running_pos(const CalibrationResult &calibration_result, const EncoderEstimatesResult &fb);

SequenceStatus position_pid(const ODriveCalibrationResult &limits, const EncoderEstimatesResult &fb,
                            const float goal_pos, const double dt);
