#include "calibration.h"

#include <Arduino.h>
#include <Wire.h>

#include "as5600_state.h"
#include "odrive_state.h"

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
			LOOP_LOG("AS5600 calibration done.\n");
			current_state = CalibrationState::Done;
		}
		break;
	}
	case CalibrationState::ODRIVE: {
		if (dt < 10000)
			break;

		last_sample_time = t;
		SequenceState result = odrive_calibration();
		if (result == SequenceState::Done) {
			LOOP_LOG("Odrive calibration done.\n");
			current_state = CalibrationState::Done;
		} else if (result == SequenceState::Error) {
			current_state = CalibrationState::Error;
		}
		break;
	}
	case CalibrationState::Done:
		return SequenceState::Done;

	case CalibrationState::Error:
		return SequenceState::Error;

	default:
		LOOP_ERROR("Calibration sequence got an unknown state.");
		current_state = CalibrationState::Error;
		break;
	}

	return SequenceState::Running;
}
