#include "src/states/running/main_sequence.h"

#include <cmath>

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"

SequenceStatus main_sequence(MainSequenceState &current_state, const CalibrationResult &calibration_result,
                             const EncoderEstimatesResult &fb) {

	static unsigned long last_sample_time = 0;
	unsigned long t = micros();
	unsigned long dt = t - last_sample_time;
	double dt_s = dt / 1'000'000.0;

	if (dt < 10'000)
		return SequenceStatus::RUNNING;

	last_sample_time = t;

	static unsigned long closed_loop_timeout = 0;

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
				current_state = MainSequenceState::ENABLE_TYPE_CONTROL;
			}
		}
		break;
	}
	case MainSequenceState::ENABLE_TYPE_CONTROL: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("[RUNNING] [MAIN] Switching to position control was not possible");
			current_state = MainSequenceState::ERROR;
		} else {
			LOOP_LOG("[RUNNING] [MAIN] Position control set succesfully");
			current_state = MainSequenceState::POSITION_PID;
		}
		break;
	}
	case MainSequenceState::SINUSOIDAL_POS: {

		SequenceStatus status = running_pos(calibration_result, fb);
		break;
	}

	case MainSequenceState::POSITION_PID: {

		SequenceStatus status =
		    position_pid(calibration_result.odrive_result, fb, calibration_result.odrive_result.midpoint + 3.0f, dt_s);

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

SequenceStatus running_pos(const CalibrationResult &calibration_result, const EncoderEstimatesResult &fb) {
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
	odrv0.setPosition(center + amplitude * sinf(phase), 0.0f);

	double as5600_rads = as5600_read_rads(calibration_result.inner_encoder_result.raw_offset);
	int raw_angle = as5600_read_raw();

	double cosine = cos(as5600_rads);
	double sine = sin(as5600_rads);

	LOOP_LOG("cosine: %.2f,\t sine: %.2f", cosine, sine);

	return SequenceStatus::RUNNING;
}

SequenceStatus position_pid(const ODriveCalibrationResult &limits, const EncoderEstimatesResult &fb,
                            const float goal_pos, const double dt) {

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = goal_pos - fb.pos;

	float p_gain = 0.5;
	float p_term = error * p_gain;

	if (first_run) {
		prev_error = error;
		first_run = false;
	}

	float d_gain = 0.05;
	float d_term = (error - prev_error) / dt * d_gain;

	prev_error = error;

	float max_torque = 0.5;

	static float integral = 0.0f;

	integral += error * dt;

	float i_gain = 0.1;
	float i_term = integral * i_gain;
	i_term = std::clamp(i_term, -max_torque, max_torque);

	float torque = p_term + d_term + i_term;
	torque = std::clamp(torque, -max_torque, max_torque);

	LOOP_LOG("goal = %.2f,\tpos = %.2f,\ttorque = %.2f,\tp_term = %.2f,\td_term = %.2f,\ti_term = %.2f", goal_pos,
	         fb.pos, torque, p_term, d_term, i_term);

	odrv0.setTorque(torque);

	return SequenceStatus::RUNNING;
}
