#include "calibration.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/states/calibration/as5600_state.h"
#include "src/states/calibration/odrive_state.h"

#include "src/utils/log_macros.h"

SequenceState calibration_sequence() {
	static CalibrationState current_state = CalibrationState::ODRIVE;

	static unsigned long last_sample_time = 0;
	unsigned long t = micros();
	unsigned long dt = t - last_sample_time;

	switch (current_state) {
	case CalibrationState::AS5600: {
		if (dt < 1000)
			break;

		last_sample_time = t;

		if (as5600_calibration()) {
			LOOP_LOG("AS5600 calibration DONE.\n");
			current_state = CalibrationState::DONE;
		}
		break;
	}
	case CalibrationState::ODRIVE: {
		if (dt < 10000)
			break;

		last_sample_time = t;
		SequenceState result = odrive_calibration();
		if (result == SequenceState::DONE) {
			LOOP_LOG("Odrive calibration DONE.\n");
			current_state = CalibrationState::DONE;
		} else if (result == SequenceState::ERROR) {
			current_state = CalibrationState::ERROR;
		}
		break;
	}
	case CalibrationState::DONE:
		return SequenceState::DONE;

	case CalibrationState::ERROR:
		return SequenceState::ERROR;

	default:
		LOOP_ERROR("Calibration sequence got an unknown state.");
		current_state = CalibrationState::ERROR;
		break;
	}

	return SequenceState::RUNNING;
}
