#include "src/states/running/setup_state.h"

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/log_macros.h"

SequenceStatus setup_sequence(const EncoderEstimatesResult &fb, const ODriveCalibrationResult &limits) {
	static RunningSetupState current_state = RunningSetupState::ENABLE_CONTROL_LOOP_CONTROL;

	static unsigned long closed_loop_timeout = 0;

	static float closest_physical_limit = 0.0f;
	static float closest_limit = 0.0f;

	pumpEvents(ESP32Can);

	switch (current_state) {
	case RunningSetupState::ENABLE_CONTROL_LOOP_CONTROL: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		closed_loop_timeout = millis();

		LOOP_LOG("[RUNNING] [SETUP] Enabling for closed loop control");
		current_state = RunningSetupState::WAIT_FOR_CONTROL_LOOP_CONTROL;

		break;
	}
	case RunningSetupState::WAIT_FOR_CONTROL_LOOP_CONTROL: {
		if (millis() - closed_loop_timeout > 1000) {
			LOOP_ERROR("[RUNNING] [SETUP] Closed loop control couldn't be enabled");
			current_state = RunningSetupState::ERROR;
			break;
		}

		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 10)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("[RUNNING] [SETUP] Closed loop control enabled after waiting");
				odrv0.setTorque(0.0f);
				odrv0.setVelocity(0.0f);
				current_state = RunningSetupState::ENABLE_POSITION_CONTROL;
			}
		}
		break;
	}
	case RunningSetupState::ENABLE_POSITION_CONTROL: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
		                             ODriveInputMode::INPUT_MODE_TRAP_TRAJ)) {
			LOOP_ERROR("[RUNNING] [SETUP] Switching to position control was not possible");
			current_state = RunningSetupState::ERROR;
		} else {
			LOOP_LOG("[RUNNING] [SETUP] Position control set succesfully");
			current_state = RunningSetupState::GO_TO_MID_POINT;
		}
		break;
	}
	case RunningSetupState::GO_TO_MID_POINT: {
		if (move_to_position(limits.midpoint, 6.0f, std::make_pair(10.0f, 10.0f))) {
			LOOP_LOG("[RUNNING] [SETUP] Reached midpoint successfully");

			current_state = RunningSetupState::DONE;
		}
		break;
	}
	case RunningSetupState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = RunningSetupState::ENABLE_CONTROL_LOOP_CONTROL;

		LOOP_LOG("[RUNNING] [SETUP] DONE");
		return SequenceStatus::DONE;
	}

	case RunningSetupState::ERROR: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = RunningSetupState::ENABLE_CONTROL_LOOP_CONTROL;
		return SequenceStatus::ERROR;
	}

	default: {
		LOOP_ERROR("[RUNNING] [SETUP] Unexpected setup state: %d", (int)current_state);
		current_state = RunningSetupState::ERROR;
		break;
	}
	}
	return SequenceStatus::RUNNING;
}
