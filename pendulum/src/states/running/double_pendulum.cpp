#include "src/states/running/double_pendulum.h"

#include <cmath>

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"
#include "src/utils/tflite.h"
#include "src/config.h"

SequenceStatus double_pendulum(DoublePendulumState &current_state, const ODriveCalibrationResult &rail_limits,
                               const EncoderEstimatesResult &fb, const float inner_rads, const float outer_rads) {

	static unsigned long last_sample_time = 0;
	unsigned long t = micros();
	unsigned long dt_us = t - last_sample_time;
	if (dt_us < 10'000)
		return SequenceStatus::RUNNING;

	float dt_s = dt_us / 1'000'000.0;

	last_sample_time = t;

	static unsigned long closed_loop_timeout = 0;
	static float smoothed_action = 0.0f;

	switch (current_state) {
	case DoublePendulumState::ENABLE_CONTROL_LOOP_CONTROL: {
		smoothed_action = 0.0f;
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
		closed_loop_timeout = millis();

		LOOP_LOG("[RUNNING] [MAIN] Enabling for closed loop control");
		current_state = DoublePendulumState::WAIT_FOR_CONTROL_LOOP_CONTROL;

		break;
	}
	case DoublePendulumState::WAIT_FOR_CONTROL_LOOP_CONTROL: {
		if (millis() - closed_loop_timeout > 1000) {
			LOOP_ERROR("[RUNNING] [MAIN] Closed loop control couldn't be enabled");
			current_state = DoublePendulumState::ERROR;
			break;
		}

		Heartbeat_msg_t hb;
		if (odrv0.request(hb, 10)) {
			if (hb.Axis_State == ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
				LOOP_LOG("[RUNNING] [MAIN] Closed loop control enabled after waiting");
				odrv0.setTorque(0.0f);
				odrv0.setVelocity(0.0f);
				current_state = DoublePendulumState::ENABLE_TYPE_CONTROL;
			}
		}
		break;
	}
	case DoublePendulumState::ENABLE_TYPE_CONTROL: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("[RUNNING] [MAIN] Switching to position control was not possible");
			current_state = DoublePendulumState::ERROR;
		} else {
			LOOP_LOG("[RUNNING] [MAIN] Position control set succesfully");
			current_state = DoublePendulumState::MONITOR_AS5600;
			// current_state = DoublePendulumState::INNER_FREE_SWING;
			// current_state = DoublePendulumState::OUTER_FREE_SWING;
			// current_state = DoublePendulumState::COAST_DOWN_TEST;
			// current_state = DoublePendulumState::BREAKAWAY_TEST;
		}
		break;
	}

	case DoublePendulumState::MONITOR_AS5600: {
		static unsigned long stable_since_ms = 0;

		if (millis() - stable_since_ms > 4000)
			stable_since_ms = millis();

		float threshold = PI + 1;
		float cos_inner = cos(inner_rads);

		if (false)
			stable_since_ms = millis();
		else if (millis() - stable_since_ms >= 3000) {
			LOOP_LOG("PENDULUM IS STABLE AND UPRIGHT");
			// current_state = DoublePendulumState::NN_BALANCING;
			// current_state = DoublePendulumState::NN_SWINGUP;
			current_state = DoublePendulumState::MOTOR_CUVRE_TEST;
			// current_state = DoublePendulumState::BREAKAWAY_TEST;
		}

		// LOOP_LOG("[AS5600] inner_angle = %.6f,\touter_angle = %.6f\n", inner_rads, outer_rads);

		break;
	}

	case DoublePendulumState::NN_BALANCING: {

		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;
		float cart_v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		if (cos(inner_rads) < cos(PI / 2.0) || abs(pos_m) > 0.4) {
			current_state = DoublePendulumState::EMERGENCY_BRAKE;
			odrv0.setTorque(0.0f);
			break;
		}

		float force_n = -double_pendulum_policy(-pos_m, -cart_v_mps, inner_rads, outer_rads, dt_s) * 40.0;
		float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[NN] force_n = %.6f,\ttorque_nm = %.6f\n", force_n, torque_nm);

		// odrv0.setTorque(torque_nm);

		break;
	}

	case DoublePendulumState::NN_SWINGUP: {

		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;
		float cart_v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		if (abs(pos_m) > 0.35) {
			current_state = DoublePendulumState::EMERGENCY_BRAKE;
			odrv0.setTorque(0.0f);
			break;
		}

		float force_n = -double_pendulum_policy(-pos_m, -cart_v_mps, inner_rads, outer_rads, dt_s) * 40.0;

		float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[NN] force_n = %.6f,\ttorque_nm = %.6f\n", force_n, torque_nm);

		// odrv0.setTorque(torque_nm);

		break;
	}

	case DoublePendulumState::BREAKAWAY_TEST: {
		static float torque_nm = 0.0f;
		static float elapsed = 0.0f;

		elapsed += dt_s;
		torque_nm = 0.005f * elapsed; // linear ramp: 0.005 Nm/s, tune this

		odrv0.setTorque(torque_nm);
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		Serial.println(String(dt_s) + "," + String(torque_nm, 6) + "," + String(v_mps, 6));

		if (abs(v_mps) > 0.5f) {
			odrv0.setTorque(0.0f);
			torque_nm = 0.0f;
			elapsed = 0.0f;
			current_state = DoublePendulumState::DONE;
		}

		break;
	}
	case DoublePendulumState::MOTOR_CUVRE_TEST: {
		static float force_n = -10.0f;
		static int step = 0;
		static float torque_nm = force_n * PULLEY_RADIUS_M;

		odrv0.setTorque(torque_nm);

		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		Serial.println(String(force_n) + "," + String(step) + "," + String(dt_s) + "," + String(pos_m) + "," +
		               String(v_mps) + "," + String(inner_rads) + "," + String(outer_rads));

		step++;
		if (pos_m < -0.25) {
			odrv0.setTorque(-torque_nm * 2.0);
			torque_nm = 0.0f;
			current_state = DoublePendulumState::EMERGENCY_BRAKE;
		}

		break;
	}
	case DoublePendulumState::COAST_DOWN_TEST: {
		float rps_for_1_mps = 1.0f / PULLEY_CIRCUMFERENCE_M; // v_mps = rps * 2 * PI * radius
		float velocity_rps = -rps_for_1_mps * 1.5f;
		odrv0.setVelocity(velocity_rps * 1.1);

		if (abs(fb.vel) > abs(velocity_rps + 0.1f)) {
			LOOP_LOG("[RUNNING] [RAMP UP] velocity matches what we expected");
			current_state = DoublePendulumState::IDLE;
		}
		break;
	}
	case DoublePendulumState::IDLE: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("[RUNNING] [MAIN] Switching to position control was not possible");
			current_state = DoublePendulumState::ERROR;
		} else {
			LOOP_LOG("[RUNNING] [MAIN] IDLE");
			current_state = DoublePendulumState::COLLECT_DATA;
		}
		break;
	}
	case DoublePendulumState::COLLECT_DATA: {
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;
		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;

		Serial.println(String(dt_s) + "," + String(pos_m, 6) + "," + String(v_mps, 6));

		if (abs(v_mps) < abs(0.05)) {
			current_state = DoublePendulumState::DONE;
		}
		break;
	}
	case DoublePendulumState::INNER_FREE_SWING: {
		static bool first_pass = true;
		static float inner_prev_rads = 0.0f;

		if (first_pass) {
			inner_prev_rads = inner_rads;
			Serial.println("time_s,pole_cos,pole_sin,pole_angle_rad,pole_vel_radps");
			first_pass = false;
			break;
		}

		// Inner variables
		float inner_cos = cos(inner_rads);
		float inner_sin = sin(inner_rads);
		float inner_cos_prev = cos(inner_prev_rads);
		float inner_sin_prev = sin(inner_prev_rads);
		float inner_angular_vel_radps =
		    ((inner_sin - inner_sin_prev) * inner_cos - (inner_cos - inner_cos_prev) * inner_sin) / dt_s;

		inner_prev_rads = inner_rads;

		Serial.println(String(dt_s) + "," + String(inner_cos, 6) + "," + String(inner_sin, 6) + "," +
		               String(inner_rads, 6) + "," + String(inner_angular_vel_radps, 6));

		break;
	}

	case DoublePendulumState::OUTER_FREE_SWING: {
		static bool first_pass = true;
		static float outer_prev_rads = 0.0f;

		if (first_pass) {
			outer_prev_rads = outer_rads;
			Serial.println("time_s,pole_cos,pole_sin,pole_angle_rad,pole_vel_radps");
			first_pass = false;
			break;
		}

		// Inner variables
		float outer_cos = cos(outer_rads);
		float outer_sin = sin(outer_rads);
		float outer_cos_prev = cos(outer_prev_rads);
		float outer_sin_prev = sin(outer_prev_rads);
		float outer_angular_vel_radps =
		    ((outer_sin - outer_sin_prev) * outer_cos - (outer_cos - outer_cos_prev) * outer_sin) / dt_s;

		outer_prev_rads = outer_rads;

		Serial.println(String(dt_s) + "," + String(outer_cos, 6) + "," + String(outer_sin, 6) + "," +
		               String(outer_rads, 6) + "," + String(outer_angular_vel_radps, 6));

		break;
	}

	case DoublePendulumState::EMERGENCY_BRAKE: {
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;
		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;

		int8_t sign = v_mps > 0.0 ? -1 : 1;

		static float force_n = 80.0f * sign;
		static float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[RUNNING] [EMERGENCY_BRAKE] v_mps = %.6f,\t force_n = %.6f", v_mps, force_n);

		if (abs(v_mps) > 0.0 && abs(pos_m) >= 0.25) {
			// if (abs(v_mps) > 0.0) {
			odrv0.setTorque(torque_nm);
		} else {
			odrv0.setTorque(0.0);
			torque_nm = 0.0f;
			current_state = DoublePendulumState::ERROR;
		}

		break;
	}

	case DoublePendulumState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = DoublePendulumState::ENABLE_CONTROL_LOOP_CONTROL;

		LOOP_LOG("[RUNNING] [MAIN] DONE");
		// return SequenceStatus::DONE;
	}

	case DoublePendulumState::ERROR: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = DoublePendulumState::ENABLE_CONTROL_LOOP_CONTROL;
		return SequenceStatus::ERROR;
	}

	default: {
		LOOP_ERROR("[RUNNING] [MAIN] Unexpected setup state: %d", (int)current_state);
		current_state = DoublePendulumState::ERROR;
		break;
	}
	}

	return SequenceStatus::RUNNING;
}

float double_pendulum_policy(const float cart_pos_m, const float cart_vel_mps, const float _inner_rads,
                             const float _outer_rads, const float dt_s) {
	static bool first_run = true;
	static float inner_prev_rads = 0.0f;
	static float outer_prev_rads = 0.0f;
	float inner_upright_offset = 0.03;
	float outer_upright_offset = 0.00;

	if (first_run) {
		inner_prev_rads = _inner_rads - inner_upright_offset;
		inner_prev_rads = _outer_rads - outer_upright_offset;
		first_run = false;
		return 0.0f;
	}

	float inner_rads = _inner_rads - inner_upright_offset;
	float outer_rads = _outer_rads - outer_upright_offset;

	// Inner variables
	float inner_cos = cos(inner_rads);
	float inner_sin = sin(inner_rads);
	float inner_cos_prev = cos(inner_prev_rads);
	float inner_sin_prev = sin(inner_prev_rads);
	float inner_angular_vel_radps =
	    ((inner_sin - inner_sin_prev) * inner_cos - (inner_cos - inner_cos_prev) * inner_sin) / dt_s;

	inner_prev_rads = inner_rads;

	// outer variables
	float outer_cos = cos(outer_rads);
	float outer_sin = sin(outer_rads);
	float outer_cos_prev = cos(outer_prev_rads);
	float outer_sin_prev = sin(outer_prev_rads);
	float outer_angular_vel_radps =
	    ((outer_sin - outer_sin_prev) * outer_cos - (outer_cos - outer_cos_prev) * outer_sin) / dt_s;

	outer_prev_rads = outer_rads;

	float obs[8] = {cart_pos_m, cart_vel_mps,           inner_sin, inner_cos, inner_angular_vel_radps, outer_sin,
	                outer_cos,  outer_angular_vel_radps};

	scale_observations(obs, 8);

	input->data.f[0] = obs[0];
	input->data.f[1] = obs[1];
	input->data.f[2] = obs[2];
	input->data.f[3] = obs[3];
	input->data.f[4] = obs[4];
	input->data.f[5] = obs[5];
	input->data.f[6] = obs[6];
	input->data.f[7] = obs[7];

	interpreter->Invoke();

	LOOP_LOG("[FUNCTION] dt_s = %.6f, cart_pos_m = %.6f,\tcart_vel_mps = %.6f,\t"
	         "inner_cos = %.6f,\tinner_sin = %.6f,\tinner_angular_vel_radps = %.6f,\t"
	         "outer_cos = %.6f,\touter_sin = %.6f,\touter_angular_vel_radps = %.6f",
	         dt_s, cart_pos_m, cart_vel_mps, inner_cos, inner_sin, inner_angular_vel_radps, outer_cos, outer_sin,
	         outer_angular_vel_radps);

	return output->data.f[0];
}
