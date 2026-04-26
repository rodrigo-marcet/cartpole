#pragma once

#include "src/utils/hfsm_types.h"

enum class CalibrationState : uint8_t { AS5600 = 1, ODRIVE = 2, Done = 10, Error = 255 };

SequenceState calibration_sequence();
