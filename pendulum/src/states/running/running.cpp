#include "src/states/running/running.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/states/running/setup_state.h"
#include "src/states/running/single_pendulum.h"
#include "src/states/running/double_pendulum.h"

#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"
#include "src/utils/odrive.h"

SequenceStatus running_sequence(const CalibrationResult &calibration_result) {
	static RunningState current_state = RunningState::SETUP;

	static SinglePendulumState single_pendulum_state = SinglePendulumState::ENABLE_CONTROL_LOOP_CONTROL;
	static DoublePendulumState double_pendulum_state = DoublePendulumState::ENABLE_CONTROL_LOOP_CONTROL;

	const ODriveCalibrationResult &limits = calibration_result.odrive_result;

	pumpEvents(ESP32Can);

	auto t_pump = micros();
	EncoderEstimatesResult fb = get_encoder_estimates();
	double inner_encoder_rads = as5600_read_rads(calibration_result.inner_encoder_result.raw_offset, INNER_AS5600_IDX);
	double outer_encoder_rads = as5600_read_rads(calibration_result.outer_encoder_result.raw_offset, OUTER_AS5600_IDX);

	inner_encoder_rads = inner_encoder_rads - PI;
	if (inner_encoder_rads < -PI)
		inner_encoder_rads += 2.0 * PI;
	if (inner_encoder_rads > PI)
		inner_encoder_rads -= 2.0 * PI;

	if (!fb.ok) {
		LOOP_ERROR("Error reading fb at the guard clause of the running sequence");
		current_state = RunningState::ERROR;
	}

	if (odrv0_user_data.last_heartbeat.Axis_Error != 0) {
		LOOP_ERROR("[RUNNING] [HEARTBEAT] odrive detected axis error: %i", odrv0_user_data.last_heartbeat.Axis_Error);
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
			// current_state = RunningState::SINGLE_PENDULUM;
			current_state = RunningState::DOUBLE_PENDULUM;
		} else if (status == SequenceStatus::ERROR) {
			LOOP_LOG("[RUNNING] [SETUP] we got an error, diverging to error state");
			current_state = RunningState::ERROR;
		}
		break;
	}

	case RunningState::SINGLE_PENDULUM: {
		SequenceStatus status =
		    single_pendulum(single_pendulum_state, calibration_result.odrive_result, fb, inner_encoder_rads);

		if (status == SequenceStatus::DONE) {
			current_state = RunningState::DONE;
		} else if (status == SequenceStatus::ERROR) {
			current_state = RunningState::ERROR;
		}
		break;
	}

	case RunningState::DOUBLE_PENDULUM: {
		SequenceStatus status = double_pendulum(double_pendulum_state, calibration_result.odrive_result, fb,
		                                        inner_encoder_rads, outer_encoder_rads);

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

		single_pendulum_state = SinglePendulumState::ENABLE_CONTROL_LOOP_CONTROL;
		single_pendulum_state = SinglePendulumState::ENABLE_CONTROL_LOOP_CONTROL;
		current_state = RunningState::SETUP;
		break;
	}

	case RunningState::DONE: {
		current_state = RunningState::SETUP;
		return SequenceStatus::DONE;
	}

	case RunningState::ERROR: {
		Get_Error_msg_t error_msg;
		if (odrv0.getError(error_msg, 10)) {
			LOOP_ERROR("[RUNNING] [ERROR] got motor error: %i", error_msg.Active_Errors);
		}
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

	// LOOP_LOG("[PROF] running: %.3f ms", (micros() - t_pump) / 1000.0);

	return SequenceStatus::RUNNING;
}
