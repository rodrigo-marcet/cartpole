#pragma once

#include "src/utils/hfsm_types.h"

enum class CalibrationState : uint8_t { AS5600, ODRIVE, DONE = 254, ERROR = 255 };

SequenceState calibration_sequence();
