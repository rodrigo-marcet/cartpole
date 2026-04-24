#include "calibration.h"

#include <Arduino.h>
#include <Wire.h>

#include "as5600_state.h"

SequenceState calibration_sequence() {
	static CalibrationState current_state = CalibrationState::Done;

	static unsigned long last_sample_time = 0;
	unsigned long t = micros();
	unsigned long dt = t - last_sample_time;

	switch (current_state) {
	case CalibrationState::AS5600:
		if (dt < 1000)
			break;

		last_sample_time = t;

		if (as5600_calibration()) {
			Serial.print("--- AS5600 calibration done.\n");
			current_state = CalibrationState::Done;
		}
		break;

	case CalibrationState::ODRIVE:
		if (dt < 1000)
			break;

		last_sample_time = t;

		if (as5600_calibration()) {
			Serial.print("--- AS5600 calibration done.\n");
			current_state = CalibrationState::Done;
		}
		break;

	case CalibrationState::Done:
		return SequenceState::Done;
		break;

	default:
		Serial.print("Calibration sequence got an unknown state.");
		return SequenceState::Error;
		break;
	}

	return SequenceState::Running;
}
