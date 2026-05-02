#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class MainSequenceState : uint8_t {
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	ENABLE_POSITION_CONTROL,
	SINUSOIDAL_POS,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus main_sequence(MainSequenceState &current_state, const EncoderEstimatesResult &fb,
                             const ODriveCalibrationResult &limits);

SequenceStatus running_sequence(const ODriveCalibrationResult &calibration_result);

SequenceStatus running_pos(const ODriveCalibrationResult &calibration_result, const EncoderEstimatesResult &fb);

SequenceStatus running_torque(const ODriveCalibrationResult &calibration_result, const EncoderEstimatesResult &fb);
