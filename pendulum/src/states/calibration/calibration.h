#pragma once

#include "src/utils/hfsm_types.h"

enum class CalibrationState : uint8_t { INNER_AS5600, OUTER_AS5600, ODRIVE, DONE = 254, ERROR = 255 };

SequenceStatus calibration_sequence(CalibrationResult *result);
