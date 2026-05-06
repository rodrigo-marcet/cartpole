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
			current_state = MainSequenceState::MONITOR_AS5600;
		}
		break;
	}
	case MainSequenceState::MONITOR_AS5600: {

		static unsigned long stable_since = 0;

		if (millis() - stable_since > 4000)
			stable_since = millis();

		double as5600_rads = as5600_read_rads(calibration_result.inner_encoder_result.raw_offset);
		double threshold = PI * 0.1;

		// LOOP_LOG("angle: %.6f,\t difference: %.6f", as5600_rads, as5600_rads - PI);

		if ((PI - threshold >= as5600_rads) || (as5600_rads >= PI + threshold))
			stable_since = millis();
		else if (millis() - stable_since >= 3000) {
			LOOP_LOG("PENDULUM IS STABLE AND UPRIGHT");
			current_state = MainSequenceState::PENDULUM_PID;
		}

		break;
	}

	case MainSequenceState::PENDULUM_PID: {
		static int call_count = 0;
		static float goal_deviation = 0.0f;

		if (call_count % 5 == 0) {
			goal_deviation = position_pid(calibration_result.odrive_result.midpoint, fb.pos, dt_s * 5);
		}
		call_count++;

		SequenceStatus status =
		    pendulum_pid(calibration_result.odrive_result, fb, calibration_result.inner_encoder_result.raw_offset, dt_s,
		                 PI - goal_deviation);

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

float position_pid(const float midpoint, const float current_pos, const double dt_s) {

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = midpoint - current_pos;

	float p_gain = 0.02;
	float p_term = error * p_gain;

	if (first_run) {
		prev_error = error;
		first_run = false;
	}

	float d_gain = 0.01;
	float d_term = (error - prev_error) / dt_s * d_gain;

	prev_error = error;

	float max_deviation = 0.2;

	static float integral = 0.0f;

	integral += error * dt_s;

	float i_gain = 0.05;
	float i_term = integral * i_gain;

	i_term = std::clamp(i_term, -max_deviation, max_deviation);

	// float deviation = p_term + d_term + i_term;
	float deviation = p_term + d_term;
	deviation = std::clamp(deviation, -max_deviation, max_deviation);

	LOOP_LOG("goal = %.2f,\tpos = %.2f,\tdeviation = %.2f,\tp_term = %.2f,\td_term = %.2f,\ti_term = %.2f", midpoint,
	         current_pos, deviation, p_term, d_term, i_term);

	return deviation;
}

SequenceStatus pendulum_pid(const ODriveCalibrationResult &limits, const EncoderEstimatesResult &fb,
                            const double as5600_offset, const double dt_s, const float goal_angle) {

	double upright_offset = 0.05;
	double as5600_rads = as5600_read_rads(as5600_offset);

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = (goal_angle + upright_offset) - as5600_rads;

	float p_gain = 0.8;
	float p_term = error * p_gain;

	if (first_run) {
		prev_error = error;
		first_run = false;
	}

	float d_gain = 0.05;
	float d_term = (error - prev_error) / dt_s * d_gain;

	prev_error = error;

	float max_torque = 5.0f;

	static float integral = 0.0f;

	integral += error * dt_s;

	float i_gain = 0.3;
	float max_integral = max_torque / i_gain;
	integral = std::clamp(integral, -max_integral, max_integral);

	float i_term = integral * i_gain;
	i_term = std::clamp(i_term, -max_torque, max_torque);

	float torque = p_term + d_term + i_term;
	torque = std::clamp(torque, -max_torque, max_torque);

	// LOOP_LOG("goal = %.2f,\tangle = %.2f,\ttorque = %.2f,\tp_term = %.2f,\td_term = %.2f,\ti_term = %.2f",
	// goal_angle,
	//          as5600_rads, torque, p_term, d_term, i_term);

	odrv0.setTorque(torque);

	return SequenceStatus::RUNNING;
}
