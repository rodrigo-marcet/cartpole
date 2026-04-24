#include "hfsm.h"
#include "hfsm_types.h"

#include <Arduino.h>

#include "states/calibration/calibration.h"
#include "states/running/running.h"
#include "src/utils/log_macros.h"

void hfsm() {
	static HFSMState current_state = HFSMState::Calibration;

	switch (current_state) {
	case HFSMState::Calibration:
		if (calibration_sequence() == SequenceState::Done) {
			LOOP_LOG("Calibration completed.");
			current_state = HFSMState::Running;
		}
		break;

	case HFSMState::Running:
		if (running_sequence() == SequenceState::Done) {
			LOOP_LOG("Running sequence exited.\n");
		}

		break;
	}
}
