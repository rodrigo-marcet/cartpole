#include "src/states/running/running.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/states/running/setup_state.h"
#include "src/states/running/main_sequence.h"

#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"
#include "src/utils/odrive.h"

// SequenceStatus running_sequence() {
// 	double rads = as5600_read_rads();
// 	double c = cos(rads);

// 	Serial.print(get_as5600_offset());
// 	Serial.print("         ");
// 	Serial.print(c, 6);
// 	Serial.print("\n");

// 	return SequenceStatus::RUNNING;
// }

SequenceStatus running_sequence(const CalibrationResult &calibration_result) {
	static RunningState current_state = RunningState::SETUP;

	static MainSequenceState main_sequence_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;

	const ODriveCalibrationResult &limits = calibration_result.odrive_result;

	pumpEvents(ESP32Can);

	EncoderEstimatesResult fb = get_encoder_estimates();
	if (!fb.ok) {
		LOOP_ERROR("Error reading fb at the guard clause of the running sequence");
		current_state = RunningState::ERROR;
	}

	static bool killswitch_active = false;

	if (!killswitch_active && (fb.pos < limits.lower_limit || fb.pos > limits.upper_limit)) {
		killswitch_active = true;

		LOOP_ERROR("[RUNNING] [KILLSWITCH] killswitch engaged, pos: %.3f, lower: %.3f, upper: %.3f", fb.pos,
		           limits.lower_limit, limits.upper_limit);

		current_state = RunningState::KILLSWITCH;
	}

	switch (current_state) {
	case RunningState::SETUP: {
		SequenceStatus status = setup_sequence(fb, limits);

		if (status == SequenceStatus::DONE) {
			killswitch_active = false;
			current_state = RunningState::MAIN_SEQUENCE;
		} else if (status == SequenceStatus::ERROR) {
			LOOP_LOG("[RUNNING] [SETUP] we got an error, diverging to error state");
			current_state = RunningState::ERROR;
		}
		break;
	}

	case RunningState::MAIN_SEQUENCE: {
		SequenceStatus status = main_sequence(main_sequence_state, calibration_result, fb);

		if (status == SequenceStatus::DONE) {
			current_state = RunningState::DONE;
		} else if (status == SequenceStatus::ERROR) {
			current_state = RunningState::ERROR;
		}
		break;
	}
	case RunningState::KILLSWITCH: {
		odrv0.setTorque(0.0f);
		odrv0.setVelocity(0.0f);
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

		main_sequence_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;
		current_state = RunningState::SETUP;
		break;
	}
	case RunningState::DONE:
		current_state = RunningState::SETUP;
		return SequenceStatus::DONE;

	case RunningState::ERROR: {
		odrv0.clearErrors();
		pumpEvents(ESP32Can);
		delay(10);
		current_state = RunningState::KILLSWITCH;
		break;
		// return SequenceStatus::ERROR;
	}

	default:
		LOOP_ERROR("Running sequence got an unknown state.");
		current_state = RunningState::ERROR;
		break;
	}

	return SequenceStatus::RUNNING;
}
