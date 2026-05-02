#include "calibration.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/states/calibration/as5600_state.h"
#include "src/states/calibration/odrive_state.h"

#include "src/utils/log_macros.h"
#include "src/utils/hfsm_types.h"

SequenceStatus calibration_sequence(CalibrationResult *result) {
	static CalibrationState current_state = CalibrationState::ODRIVE;

	switch (current_state) {
	case CalibrationState::AS5600: {
		SequenceStatus status = as5600_calibration(&result->inner_encoder_result);

		if (status == SequenceStatus::DONE) {
			LOOP_LOG("AS5600 calibration DONE.\n");
			current_state = CalibrationState::DONE;
		} else if (status == SequenceStatus::ERROR) {
			current_state = CalibrationState::ERROR;
		}
		break;
	}
	case CalibrationState::ODRIVE: {
		SequenceStatus status = odrive_calibration(&result->odrive_result);

		if (status == SequenceStatus::DONE) {
			LOOP_LOG("Odrive calibration DONE.\n");
			current_state = CalibrationState::DONE;
		} else if (status == SequenceStatus::ERROR) {
			current_state = CalibrationState::ERROR;
		}
		break;
	}
	case CalibrationState::DONE:
		return SequenceStatus::DONE;

	case CalibrationState::ERROR:
		return SequenceStatus::ERROR;

	default:
		LOOP_ERROR("Calibration sequence got an unknown state.");
		current_state = CalibrationState::ERROR;
		break;
	}

	return SequenceStatus::RUNNING;
}
