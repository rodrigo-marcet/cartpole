#pragma once

#include "../../hfsm_types.h"

enum class CalibrationState {
    AS5600 = 1,
    Done = 2
};

SequenceState calibration_sequence();