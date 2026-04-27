#include "src/states/calibration/odrive_state.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/setup/odrive_setup.h"

#include "src/utils/log_macros.h"
#include "src/utils/odrive.h"

constexpr float CALIBRATION_VELOCITY = 2.0f;
constexpr float VELOCITY_THRESHOLD = 0.5f;
constexpr float STATIC_VELOCITY_DEVIATION = 0.1f;
constexpr float POSITION_DEVIATION = 0.1f;
constexpr float SAFETY_TRHESHOLD = 0.03;

SequenceState odrive_calibration() {
	static OdriveCalibrationState current_state = OdriveCalibrationState::SAVE_INIT_POS;
	static float init_pos = 0.0f;
	static float midpoint = 0.0f;
	static float upper_limit = 0.0f;
	static float lower_limit = 0.0f;

	switch (current_state) {
	case OdriveCalibrationState::SAVE_INIT_POS: {
		EncoderEstimatesResult res = get_encoder_estimates();
		if (res.ok) {
			init_pos = res.pos;
			LOOP_LOG("init_pos = %.2f", init_pos);
			current_state = OdriveCalibrationState::ENABLE_VELOCITY_CONTROL_POSITIVE;
		}
		break;
	}

	case OdriveCalibrationState::ENABLE_VELOCITY_CONTROL_POSITIVE: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("Failed to enable velocity control (positive)");
			current_state = OdriveCalibrationState::ERROR;
		} else {
			LOOP_LOG("Velocity control (positive) enabled");
			current_state = OdriveCalibrationState::MOVE_TO_POSITIVE_LIMIT;
		}
		break;
	}

	case OdriveCalibrationState::MOVE_TO_POSITIVE_LIMIT: {
		if (move_to_limit(Direction::POSITIVE)) {
			odrv0.setVelocity(0.0f, 0.0f);
			LOOP_LOG("Reached positive limit");
			current_state = OdriveCalibrationState::SAVE_UPPER_LIMIT;
		}
		break;
	}

	case OdriveCalibrationState::SAVE_UPPER_LIMIT: {
		EncoderEstimatesResult res = get_encoder_estimates();
		if (res.ok) {
			upper_limit = res.pos;
			LOOP_LOG("upper_limit = %.2f", upper_limit);

			current_state = OdriveCalibrationState::MANAGE_ERRORS_1;
		}

		break;
	}
	case OdriveCalibrationState::MANAGE_ERRORS_1: {
		odrv0.clearErrors();
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

		LOOP_LOG("ERRORs managed succesfully (1)");
		current_state = OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_1;
		break;
	}
	case OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_1: {
		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 1)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("Closed loop confirmed (1)");
				current_state = OdriveCalibrationState::ENABLE_POSITION_CONTROL_1;
			}
		}
		break;
	}
	case OdriveCalibrationState::ENABLE_POSITION_CONTROL_1: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
		                             ODriveInputMode::INPUT_MODE_TRAP_TRAJ)) {
			LOOP_ERROR("Switching to position control on ENABLE_POSITION_CONTROL_1 was not possible");
			current_state = OdriveCalibrationState::ERROR;
		} else {
			LOOP_LOG("Position control (1) set succesfully");

			current_state = OdriveCalibrationState::GO_TO_INIT_POS;
		}
		break;
	}
	case OdriveCalibrationState::GO_TO_INIT_POS: {
		if (move_to_position(init_pos)) {
			LOOP_LOG("Reached init_pos successfully");

			current_state = OdriveCalibrationState::ENABLE_VELOCITY_CONTROL_NEGATIVE;
		}

		break;
	}
	case OdriveCalibrationState::ENABLE_VELOCITY_CONTROL_NEGATIVE: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("Failed to enable velocity control (positive)");
			current_state = OdriveCalibrationState::ERROR;
		} else {
			LOOP_LOG("Velocity control (positive) enabled");
			current_state = OdriveCalibrationState::MOVE_TO_NEGATIVE_LIMIT;
		}

		break;
	}

	case OdriveCalibrationState::MOVE_TO_NEGATIVE_LIMIT: {
		if (move_to_limit(Direction::NEGATIVE)) {
			odrv0.setVelocity(0.0f, 0.0f);
			LOOP_LOG("Reached negative limit");
			current_state = OdriveCalibrationState::SAVE_LOWER_LIMIT;
		}

		break;
	}

	case OdriveCalibrationState::SAVE_LOWER_LIMIT: {
		EncoderEstimatesResult res = get_encoder_estimates();
		if (res.ok) {
			lower_limit = res.pos;
			LOOP_LOG("lower_limit = %.2f", lower_limit);

			current_state = OdriveCalibrationState::MANAGE_ERRORS_2;
		}

		break;
	}
	case OdriveCalibrationState::MANAGE_ERRORS_2: {
		odrv0.clearErrors();
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

		LOOP_LOG("ERRORs managed succesfully (1)");
		current_state = OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_2;
		break;
	}
	case OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_2: {
		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 1)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("Closed loop confirmed (2)");
				current_state = OdriveCalibrationState::CALCULATE_VALUES;
			}
		}
		break;
	}

	case OdriveCalibrationState::CALCULATE_VALUES: {
		RailLimits limits = calculate_limits(lower_limit, upper_limit);
		if (limits.ok) {
			midpoint = limits.midpoint;
			lower_limit = limits.lower_limit;
			upper_limit = limits.upper_limit;

			LOOP_LOG("midpoint = %.2f, upper_limit = %.2f, lower_limit = %.2f", midpoint, upper_limit, lower_limit);
			current_state = OdriveCalibrationState::ENABLE_POSITION_CONTROL_2;
		} else
			current_state = OdriveCalibrationState::ERROR;

		break;
	}

	case OdriveCalibrationState::ENABLE_POSITION_CONTROL_2: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
		                             ODriveInputMode::INPUT_MODE_TRAP_TRAJ)) {
			LOOP_ERROR("Switching to position control on ENABLE_POSITION_CONTROL_2 was not possible");
			current_state = OdriveCalibrationState::ERROR;
		} else {
			LOOP_LOG("Position control (2) set succesfully");
			current_state = OdriveCalibrationState::GO_TO_MID_POINT;
		}
		break;
	}

	case OdriveCalibrationState::GO_TO_MID_POINT: {
		if (move_to_position(midpoint)) {
			LOOP_LOG("Reached midpoint successfully");

			current_state = OdriveCalibrationState::DONE;
		}

		break;
	}

	case OdriveCalibrationState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		LOOP_LOG("Succesfully calibrated rail dimensions");
		return SequenceState::DONE;
	}

	case OdriveCalibrationState::ERROR: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		return SequenceState::ERROR;
	}

	default: {
		LOOP_ERROR("Unexpected calibration state: %d", (int)current_state);
		current_state = OdriveCalibrationState::ERROR;
		break;
	}
	}
	return SequenceState::RUNNING;
}

bool move_to_limit(Direction direction) {
	static bool moving = false;
	int8_t d = direction == Direction::POSITIVE ? 1 : -1;

	odrv0.setVelocity(CALIBRATION_VELOCITY * d, 0.0f);

	EncoderEstimatesResult res = get_encoder_estimates();

	if (!moving && fabsf(res.vel) > VELOCITY_THRESHOLD) {
		moving = true;
	}

	if (moving && fabsf(res.vel) < VELOCITY_THRESHOLD) {
		moving = false;
		return true;
	}

	return false;
}

bool move_to_position(float position) {
	odrv0.setTrapezoidalVelLimit(6.0f);
	odrv0.setTrapezoidalAccelLimits(10.0f, 10.0f);
	odrv0.setPosition(position, 0.0f);

	EncoderEstimatesResult res = get_encoder_estimates();
	// LOOP_LOG("pos: %.2f, vel: %.2f", res.pos, res.vel);
	if (fabsf(res.vel) < STATIC_VELOCITY_DEVIATION && fabsf(res.pos - position) <= POSITION_DEVIATION) {
		return true;
	}

	return false;
}

RailLimits calculate_limits(float lower_limit, float upper_limit) {
	RailLimits result = {false, 0.0f, 0.0f, 0.0f};
	if (lower_limit >= upper_limit) {
		LOOP_ERROR("lower_limit was set higher or equal to upper_limit");
		return result;
	}

	result.midpoint = (lower_limit + upper_limit) / 2.0f;
	float total_length = upper_limit - lower_limit;
	float safety_factor = total_length * SAFETY_TRHESHOLD;
	result.lower_limit = lower_limit + safety_factor;
	result.upper_limit = upper_limit - safety_factor;
	result.ok = true;

	return result;
}

// bool odrive_calibration() {
// 	// delay(10);
// 	pumpEvents(ESP32Can);

// 	// Official example motion: sine position with velocity feedforward
// 	// const float SINE_PERIOD_S = 10.0f;
// 	// float t = 0.001f * millis();
// 	// float phase = t * (TWO_PI / SINE_PERIOD_S);

// 	// odrv0.setPosition(sinf(phase),                           // position (turns)
// 	//                   cosf(phase) * (TWO_PI / SINE_PERIOD_S) // velocity feedforward
// 	// );

// 	const float SINE_PERIOD_S = 2.0f;
// 	float t = 0.001f * millis();
// 	float phase = t * (TWO_PI / SINE_PERIOD_S);

// 	float torque_cmd = -sinf(phase) * 0.;
// 	odrv0.setTorque(torque_cmd);
// 	// pumpEvents(ESP32Can);

// 	// Print position & velocity for Serial Plotter
// 	Get_Encoder_Estimates_msg_t fb;
// 	if (odrv0.request(fb, 1)) { // 100ms timeout
// 		// LOOP_LOG("pos: %.2f		vel: %.2f", fb.Pos_Estimate, fb.Vel_Estimate);
// 		// LOOP_LOG("millis: %i", millis());
// 	}

// 	return false;
// }
