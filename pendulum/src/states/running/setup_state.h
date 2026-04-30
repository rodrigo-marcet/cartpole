#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class RunningSetupState : uint8_t {
	SET_AXIS_STATE_IDLE,
	RESET_ODRIVE_VALUES,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus setup_sequence();
