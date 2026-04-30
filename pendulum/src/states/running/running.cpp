#include "src/states/running/running.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/states/running/setup_state.h"
#include "src/states/running/killswitch_limits_state.h"

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

	static unsigned long last_sample_time = 0;
	unsigned long t = micros();
	unsigned long dt = t - last_sample_time;

	pumpEvents(ESP32Can);

	EncoderEstimatesResult fb = get_encoder_estimates();
	if (!fb.ok) {
		LOOP_ERROR("Error reading fb at the guard clause of the running sequence");
		current_state = RunningState::ERROR;
	}

	const ODriveCalibrationResult &limits = calibration_result.odrive_result;
	if (fb.pos < limits.lower_limit || fb.pos > limits.upper_limit) {
		LOOP_LOG("Position %.2f out of bounds [%.2f, %.2f]", fb.pos, limits.lower_limit, limits.upper_limit);
		current_state = RunningState::KILLSWITCH_LIMITS;
	}

	switch (current_state) {
		// case RunningState::SETUP: {
		// 	if (dt < 10000)
		// 		break;

		// 	last_sample_time = t;

		// 	SequenceStatus status = setup_sequence();

		// 	if (status == SequenceStatus::DONE) {
		// 		LOOP_LOG("Odrive calibration DONE.\n");
		// 		current_state = RunningState::DONE;
		// 	} else if (status == SequenceStatus::ERROR) {
		// 		current_state = RunningState::ERROR;
		// 	}
		// 	break;
		// }

	case RunningState::SETUP: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 1)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("Closed loop confirmed running");
			}
		}
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
		                             ODriveInputMode::INPUT_MODE_TRAP_TRAJ)) {
			LOOP_ERROR("Switching to torque control on RUNNING was not possible");
			current_state = RunningState::ERROR;
		} else {
			LOOP_LOG("Position control (2) set succesfully");
		}

		LOOP_LOG("Motor now in loop control");
		current_state = RunningState::RUNNING;
		break;
	}

	case RunningState::RUNNING: {
		if (dt < 10000)
			break;

		last_sample_time = t;

		SequenceStatus status = running_pos(calibration_result, fb);
		LOOP_LOG("lower: %.2f, upper: %.2f", limits.lower_limit, limits.upper_limit);

		if (status == SequenceStatus::DONE) {
			LOOP_LOG("Running done\n");
			current_state = RunningState::DONE;
		} else if (status == SequenceStatus::ERROR) {
			current_state = RunningState::ERROR;
		}
		break;
	}
	case RunningState::KILLSWITCH_LIMITS: {
		const ODriveCalibrationResult &limits = calibration_result.odrive_result;

		SequenceStatus status = killswitch_limits_sequence(fb, limits);

		if (status == SequenceStatus::DONE) {
			LOOP_LOG("Killswitch limit disengaged\n");
			current_state = RunningState::SETUP;
		} else if (status == SequenceStatus::ERROR) {
			current_state = RunningState::ERROR;
		}
		break;
	}
	case RunningState::DONE:
		return SequenceStatus::DONE;

	case RunningState::ERROR:
		return SequenceStatus::ERROR;

	default:
		LOOP_ERROR("Running sequence got an unknown state.");
		current_state = RunningState::ERROR;
		break;
	}

	return SequenceStatus::RUNNING;
}

SequenceStatus running_pos(const CalibrationResult &calibration_result, const EncoderEstimatesResult &fb) {

	delay(10);

	const float MARGIN = 0.1f;
	const float upper = calibration_result.odrive_result.upper_limit - MARGIN;
	const float lower = calibration_result.odrive_result.lower_limit + MARGIN;

	const float center = (upper + lower) / 2.0f;
	const float amplitude = (upper - lower) / 2.0f;

	const float SINE_PERIOD_S = 1.0f;
	float t = 0.001f * millis();
	float phase = t * (TWO_PI / SINE_PERIOD_S);

	odrv0.setTrapezoidalVelLimit(60.0f);
	odrv0.setTrapezoidalAccelLimits(250.0f, 250.0f);
	odrv0.setPosition(center + amplitude * sinf(phase), amplitude * cosf(phase) * (TWO_PI / SINE_PERIOD_S));

	LOOP_LOG("pos: %.2f, vel: %.2f", fb.pos, fb.vel);

	return SequenceStatus::RUNNING;
}

SequenceStatus running_torque(const CalibrationResult &calibration_result, const EncoderEstimatesResult &fb) {

	delay(10);

	const float SINE_PERIOD_S = 1.0f;
	static unsigned long start_time = 0;
	if (start_time == 0)
		start_time = millis();

	float t = 0.001f * (millis() - start_time);
	float phase = t * (TWO_PI / SINE_PERIOD_S);

	const float MAX_TORQUE = 0.04f;
	float torque = MAX_TORQUE * sinf(phase);

	odrv0.setTorque(torque);

	LOOP_LOG("pos: %.2f, vel: %.2f, torque: %.2f", fb.pos, fb.vel, torque);

	return SequenceStatus::RUNNING;
}
// SequenceStatus running_sequence(const CalibrationResult& calibration_result) {

// 	delay(10);

// 	const float center    = (calibration_result.odrive_result.upper_limit +
// 							calibration_result.odrive_result.lower_limit) / 2.0f;
// 	const float amplitude = (calibration_result.odrive_result.upper_limit -
// 							calibration_result.odrive_result.lower_limit) / 2.0f;

// 	const float SINE_PERIOD_S = 2.0f;
// 	float t     = 0.001f * millis();
// 	float phase = t * (TWO_PI / SINE_PERIOD_S);

// 	odrv0.setPosition(
// 		center + amplitude * sinf(phase),
// 		amplitude * cosf(phase) * (TWO_PI / SINE_PERIOD_S)
// 	);

// 	Get_Encoder_Estimates_msg_t fb;
// 	if (odrv0.request(fb, 10)) { // 100ms timeout
// 		LOOP_LOG("pos: %.2f", fb.Pos_Estimate);
// 		LOOP_LOG("vel: %.2f", fb.Vel_Estimate);
// 		LOOP_LOG("millis: %i", millis());
// 	}
// 	// // delay(10);
// 	// pumpEvents(ESP32Can);

// 	// // Official example motion: sine position with velocity feedforward
// 	// const float SINE_PERIOD_S = 2.0f;
// 	// float t = 0.001f * millis();
// 	// float phase = t * (TWO_PI / SINE_PERIOD_S);

// 	// odrv0.setPosition(sinf(phase),                           // position (turns)
// 	//                   cosf(phase) * (TWO_PI / SINE_PERIOD_S) // velocity feedforward
// 	// );

// 	// // Print position & velocity for Serial Plotter
// 	// Get_Encoder_Estimates_msg_t fb;
// 	// if (odrv0.request(fb, 10)) { // 100ms timeout
// 	// 	LOOP_LOG("pos: %.2f", fb.Pos_Estimate);
// 	// 	LOOP_LOG("vel: %.2f", fb.Vel_Estimate);
// 	// 	LOOP_LOG("millis: %i", millis());
// 	// }

// 	return SequenceStatus::RUNNING;
// }
