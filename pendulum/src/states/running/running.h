#pragma once

#include "src/utils/hfsm_types.h"
#include "src/utils/odrive.h"

enum class RunningState : uint8_t { SETUP, RUNNING, MANUAL_RESTART, KILLSWITCH, DONE = 254, ERROR = 255 };

SequenceStatus running_sequence(const CalibrationResult &calibration_result);

SequenceStatus running_pos(const CalibrationResult &calibration_result, const EncoderEstimatesResult &fb);

SequenceStatus running_torque(const CalibrationResult &calibration_result, const EncoderEstimatesResult &fb);
