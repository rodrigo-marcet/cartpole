#include "src/states/running/main_sequence.h"

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/log_macros.h"

SequenceStatus main_sequence(const EncoderEstimatesResult &fb, const ODriveCalibrationResult &limits) {
	static MainSequenceState current_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;

	static unsigned long closed_loop_timeout = 0;

	static float closest_physical_limit = 0.0f;
	static float closest_limit = 0.0f;

	pumpEvents(ESP32Can);

	switch (current_state) {
	case MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		closed_loop_timeout = millis();

		LOOP_LOG("[RUNNING] [MAIN] Enabling for closed loop control");
		current_state = MainSequenceState::WAIT_FOR_CONTROL_LOOP_CONTROL;

		break;
	}
	case MainSequenceState::WAIT_FOR_CONTROL_LOOP_CONTROL: {
		if (millis() - closed_loop_timeout > 1000) {
			LOOP_ERROR("[RUNNING] [MAIN] Closed loop control couldn't be enabled");
			current_state = MainSequenceState::ERROR;
			break;
		}

		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 10)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("[RUNNING] [MAIN] Closed loop control enabled after waiting");
				odrv0.setTorque(0.0f);
				odrv0.setVelocity(0.0f);
				current_state = MainSequenceState::ENABLE_POSITION_CONTROL;
			}
		}
		break;
	}
	case MainSequenceState::ENABLE_POSITION_CONTROL: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("[RUNNING] [MAIN] Switching to position control was not possible");
			current_state = MainSequenceState::ERROR;
		} else {
			LOOP_LOG("[RUNNING] [MAIN] Position control set succesfully");
			current_state = MainSequenceState::SINUSOIDAL_POS;
		}
		break;
	}
	case MainSequenceState::SINUSOIDAL_POS: {

		// SequenceStatus status = running_pos(limits, fb);
		odrv0.setVelocity(10.0f);

		break;
	}
	case MainSequenceState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;

		LOOP_LOG("[RUNNING] [MAIN] DONE");
		return SequenceStatus::DONE;
	}

	case MainSequenceState::ERROR: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;
		return SequenceStatus::ERROR;
	}

	default: {
		LOOP_ERROR("[RUNNING] [MAIN] Unexpected setup state: %d", (int)current_state);
		current_state = MainSequenceState::ERROR;
		break;
	}
	}
	return SequenceStatus::RUNNING;
}

SequenceStatus running_pos(const ODriveCalibrationResult &limits, const EncoderEstimatesResult &fb) {
	const float MARGIN = 0.1f;
	const float upper = limits.upper_limit - MARGIN;
	const float lower = limits.lower_limit + MARGIN;

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
