#include "src/states/calibration/odrive_state.h"

#include <Arduino.h>
#include <Wire.h>

#include "src/utils/odrive_types.h"
#include "src/utils/log_macros.h"
#include "src/utils/odrive.h"

constexpr float CALIBRATION_VELOCITY = 2.0f;
constexpr float VELOCITY_THRESHOLD = 0.5f;
constexpr float SAFETY_THRESHOLD = 0.05;

SequenceStatus odrive_calibration(ODriveCalibrationResult *result) {
	static OdriveCalibrationState current_state = OdriveCalibrationState::SAVE_INIT_POS;

	static float closed_loop_timeout = 0.0f;

	static float init_pos = 0.0f;
	static float midpoint = 0.0f;
	static float physical_upper_limit = 0.0f;
	static float physical_lower_limit = 0.0f;

	pumpEvents(ESP32Can);

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
			physical_upper_limit = res.pos;
			LOOP_LOG("physical_upper_limit = %.2f", physical_upper_limit);

			current_state = OdriveCalibrationState::MANAGE_ERRORS_1;
		}

		break;
	}
	case OdriveCalibrationState::MANAGE_ERRORS_1: {
		odrv0.clearErrors();
		pumpEvents(ESP32Can);
		delay(10);
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		closed_loop_timeout = millis();
		LOOP_LOG("Errors managed succesfully (1)");
		current_state = OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_1;
		break;
	}
	case OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_1: {
		if (millis() - closed_loop_timeout > 100) {
			LOOP_ERROR("Closed loop control couldn't be enabled (1)");
			current_state = OdriveCalibrationState::ERROR;
			break;
		}

		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 10)) {
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
		if (move_to_position(midpoint, 6.0f, std::make_pair(10.0f, 10.0f))) {
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
			physical_lower_limit = res.pos;
			LOOP_LOG("physical_lower_limit = %.2f", physical_lower_limit);

			current_state = OdriveCalibrationState::MANAGE_ERRORS_2;
		}

		break;
	}
	case OdriveCalibrationState::MANAGE_ERRORS_2: {
		odrv0.clearErrors();
		pumpEvents(ESP32Can);
		delay(10);
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		closed_loop_timeout = millis();

		LOOP_LOG("Errors managed succesfully (2)");
		current_state = OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_2;
		break;
	}
	case OdriveCalibrationState::WAIT_FOR_CLOSED_LOOP_2: {
		if (millis() - closed_loop_timeout > 100) {
			LOOP_ERROR("Closed loop control couldn't be enabled (2)");
			current_state = OdriveCalibrationState::ERROR;
			break;
		}

		Heartbeat_msg_t hb;

		if (odrv0.request(hb, 10)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("Closed loop confirmed (2)");
				current_state = OdriveCalibrationState::CALCULATE_VALUES;
			}
		}
		break;
	}

	case OdriveCalibrationState::CALCULATE_VALUES: {
		RailLimits limits = calculate_limits(physical_lower_limit, physical_upper_limit);
		if (limits.ok) {
			midpoint = limits.midpoint;

			result->midpoint = midpoint;
			result->lower_limit = limits.lower_limit;
			result->upper_limit = limits.upper_limit;
			result->physical_lower_limit = physical_lower_limit;
			result->physical_upper_limit = physical_upper_limit;

			LOOP_LOG("midpoint = %.2f, upper_limit = %.2f, lower_limit = %.2f", midpoint, limits.upper_limit,
			         limits.lower_limit);
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
		if (move_to_position(midpoint, 6.0f, std::make_pair(10.0f, 10.0f))) {
			LOOP_LOG("Reached midpoint successfully");

			current_state = OdriveCalibrationState::DONE;
		}

		break;
	}

	case OdriveCalibrationState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

		LOOP_LOG("Succesfully calibrated rail dimensions");
		return SequenceStatus::DONE;
	}

	case OdriveCalibrationState::ERROR: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		return SequenceStatus::ERROR;
	}

	default: {
		LOOP_ERROR("Unexpected calibration state: %d", (int)current_state);
		current_state = OdriveCalibrationState::ERROR;
		break;
	}
	}
	return SequenceStatus::RUNNING;
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

RailLimits calculate_limits(float lower_limit, float upper_limit) {
	RailLimits result = {false, 0.0f, 0.0f, 0.0f};
	if (lower_limit >= upper_limit) {
		LOOP_ERROR("lower_limit was set higher or equal to upper_limit");
		return result;
	}

	result.midpoint = (lower_limit + upper_limit) / 2.0f;
	float total_length = upper_limit - lower_limit;
	float safety_factor = total_length * SAFETY_THRESHOLD;
	result.lower_limit = lower_limit + safety_factor;
	result.upper_limit = upper_limit - safety_factor;
	result.ok = true;

	return result;
}
