// #include "src/states/running/setup_state.h"

// #include "src/utils/odrive.h"
// #include "src/utils/odrive_types.h"
// #include "src/utils/log_macros.h"

// SequenceStatus odrive_calibration(ODriveCalibrationResult *result) {
// 	static RunningSetupState current_state = RunningSetupState::SAVE_INIT_POS;

// 	static float init_pos = 0.0f;
// 	static float midpoint = 0.0f;
// 	static float upper_limit = 0.0f;
// 	static float lower_limit = 0.0f;

// 	pumpEvents(ESP32Can);

// 	switch (current_state) {
// 	case RunningSetupState::RESET_ODRIVE_VALUES: {
//         break;
//     }
//         	case RunningSetupState::RESET_ODRIVE_VALUES: {
// 		odrv0.clearErrors();
// 		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

// 		LOOP_LOG("Errors managed succesfully (2)");
// 		current_state = RunningSetupState::WAIT_FOR_CLOSED_LOOP_2;
// 		break;
// 	}
// 	case RunningSetupState::WAIT_FOR_CLOSED_LOOP_2: {
// 		Heartbeat_msg_t hb;
// 		if (odrv0.request(hb, 1)) {
// 			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
// 				LOOP_LOG("Closed loop confirmed (2)");
// 				current_state = RunningSetupState::CALCULATE_VALUES;
// 			}
// 		}
// 		break;
// 	}

// 	case RunningSetupState::ENABLE_POSITION_CONTROL_2: {
// 		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_POSITION_CONTROL,
// 		                             ODriveInputMode::INPUT_MODE_TRAP_TRAJ)) {
// 			LOOP_ERROR("Switching to position control on ENABLE_POSITION_CONTROL_2 was not possible");
// 			current_state = RunningSetupState::ERROR;
// 		} else {
// 			LOOP_LOG("Position control (2) set succesfully");
// 			current_state = RunningSetupState::GO_TO_MID_POINT;
// 		}
// 		break;
// 	}

// 	case RunningSetupState::GO_TO_MID_POINT: {
// 		if (move_to_position(midpoint)) {
// 			LOOP_LOG("Reached midpoint successfully");

// 			current_state = RunningSetupState::DONE;
// 		}

// 		break;
// 	}

// 	case RunningSetupState::DONE: {
// 		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);

// 		result->midpoint = midpoint;
// 		result->upper_limit = upper_limit;
// 		result->lower_limit = lower_limit;

// 		LOOP_LOG("Succesfully calibrated rail dimensions");
// 		return SequenceStatus::DONE;
// 	}

// 	case RunningSetupState::ERROR: {
// 		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
// 		return SequenceStatus::ERROR;
// 	}

// 	default: {
// 		LOOP_ERROR("Unexpected calibration state: %d", (int)current_state);
// 		current_state = RunningSetupState::ERROR;
// 		break;
// 	}
// 	}
// 	return SequenceStatus::RUNNING;
// }
