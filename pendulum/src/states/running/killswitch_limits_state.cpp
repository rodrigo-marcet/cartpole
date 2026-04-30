#include "src/states/running/killswitch_limits_state.h"

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/log_macros.h"

SequenceStatus killswitch_limits_sequence(const EncoderEstimatesResult &fb, const ODriveCalibrationResult &limits) {
	static KillswitchLimitsState current_state = KillswitchLimitsState::CHECK_CONTROL_LOOP_CONTROL;

	static unsigned long closed_loop_timeout = 0;

	static float closest_limit = 0.0f;

	pumpEvents(ESP32Can);

	switch (current_state) {
	case KillswitchLimitsState::CHECK_CONTROL_LOOP_CONTROL: {
		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 1)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("KILLSWITCH LIMITS Closed loop confirmed");
				current_state = KillswitchLimitsState::CHECK_VELOCITY;
			} else {
				LOOP_LOG("KILLSWITCH LIMITS Closed loop not enabled");
				current_state = KillswitchLimitsState::ENABLE_CONTROL_LOOP_CONTROL;
			}
		}
		break;
	}
	case KillswitchLimitsState::ENABLE_CONTROL_LOOP_CONTROL: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		closed_loop_timeout = millis();

		LOOP_LOG("KILLSWITCH LIMITS Enabling for closed loop control");
		current_state = KillswitchLimitsState::WAIT_FOR_CONTROL_LOOP_CONTROL;

		break;
	}
	case KillswitchLimitsState::WAIT_FOR_CONTROL_LOOP_CONTROL: {
		if (millis() - closed_loop_timeout > 100) {
			LOOP_ERROR("KILLSWITCH LIMITS Closed loop control couldn't be enabled");
			current_state = KillswitchLimitsState::CHECK_VELOCITY;
			break;
		}

		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 1)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("KILLSWITCH LIMITS Closed loop control enabled after waiting");
				current_state = KillswitchLimitsState::CHECK_VELOCITY;
			}
		}
		break;
	}
	case KillswitchLimitsState::CHECK_VELOCITY: {
		if (fb.pos >= limits.upper_limit)
			closest_limit = limits.upper_limit;
		else
			closest_limit = limits.lower_limit;

		// if velocity is larger than 1t/s and we are moving towards the closest limit
		if (fabs(fb.vel) > 1.0f && (fb.vel * closest_limit > 0.0f)) {
			LOOP_LOG("KILLSWITCH LIMITS Going way too fast for position control, enabling torque control");
			current_state = KillswitchLimitsState::ENABLE_TORQUE_CONTROL;
		} else {
			LOOP_LOG("KILLSWITCH LIMITS Going at a low enough speed to use position control");
			current_state = KillswitchLimitsState::GO_TO_CLOSEST_LIMIT;
		}
		break;
	}

	case KillswitchLimitsState::ENABLE_TORQUE_CONTROL: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_LOG("KILLSWITCH LIMITS Torque control couldn't be enabled");
			current_state = KillswitchLimitsState::ERROR;
		} else {
			LOOP_ERROR("KILLSWITCH LIMITS Torque control enabled after waiting");

			current_state = KillswitchLimitsState::BREAK;
		}
		break;
	}
	case KillswitchLimitsState::BREAK: {
		if (fb.vel * closest_limit > 0.0f) {
			// TODO break
		} else {
			odrv0.setTorque(0.0f);
			LOOP_LOG("KILLSWITCH LIMITS break completed");
			current_state = KillswitchLimitsState::GO_TO_CLOSEST_LIMIT;
		}
		break;
	}
	case KillswitchLimitsState::GO_TO_CLOSEST_LIMIT: {
		if (move_to_position(closest_limit, 6.0f, std::make_pair(10.0f, 10.0f))) {
			LOOP_LOG("Reached midpoint successfully");

			current_state = KillswitchLimitsState::DONE;
		}
		break;
	}
	case KillswitchLimitsState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

		LOOP_LOG("Succesfully calibrated rail dimensions");
		return SequenceStatus::DONE;
	}

	case KillswitchLimitsState::ERROR: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		return SequenceStatus::ERROR;
	}

	default: {
		LOOP_ERROR("Unexpected calibration state: %d", (int)current_state);
		current_state = KillswitchLimitsState::ERROR;
		break;
	}
	}
	return SequenceStatus::RUNNING;
}
