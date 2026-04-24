#pragma once

#include "../../hfsm_types.h"

enum class CalibrationState {
	AS5600 = 1,
	ODRIVE = 2,
	Done = 10,
};

SequenceState calibration_sequence();
