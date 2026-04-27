#include "hfsm.h"
#include "src/utils/hfsm_types.h"

#include <Arduino.h>

#include "src/states/calibration/calibration.h"
#include "states/running/running.h"
#include "src/utils/log_macros.h"

void hfsm() {
	static HFSMState current_state = HFSMState::CALIBRATION;

	switch (current_state) {
	case HFSMState::CALIBRATION:
		if (calibration_sequence() == SequenceState::DONE) {
			LOOP_LOG("Calibration completed.");
			current_state = HFSMState::RUNNING;
		}
		break;

	case HFSMState::RUNNING:
		if (running_sequence() == SequenceState::DONE) {
			LOOP_LOG("Running sequence exited.\n");
		}

		break;
	}
}
