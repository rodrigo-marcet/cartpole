#include "src/hfsm.h"

#include <Arduino.h>

#include "src/states/calibration/calibration.h"
#include "src/states/running/running.h"

#include "src/utils/hfsm_types.h"
#include "src/utils/log_macros.h"

void hfsm() {
	static HFSMState current_state = HFSMState::CALIBRATION;
	static CalibrationResult calibration_result;

	switch (current_state) {
	case HFSMState::CALIBRATION: {
		SequenceStatus status = calibration_sequence(&calibration_result);

		if (status == SequenceStatus::DONE) {
			LOOP_LOG("Calibration completed.");
			current_state = HFSMState::RUNNING;
		}
		break;
	}

	case HFSMState::RUNNING: {
		SequenceStatus status = running_sequence(calibration_result);

		if (status == SequenceStatus::DONE) {
			LOOP_LOG("Running sequence exited.\n");
		}

		break;
	}
	}
}
