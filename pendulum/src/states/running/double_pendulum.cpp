#include "src/states/running/double_pendulum.h"

#include <cmath>

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"
#include "src/utils/tflite.h"
#include "src/config.h"

SequenceStatus double_pendulum(DoublePendulumState &current_state, const ODriveCalibrationResult &rail_limits,
                               const EncoderEstimatesResult &fb, const float inner_angle_rads) {

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
			// current_state = DoublePendulumState::MONITOR_AS5600;
			current_state = DoublePendulumState::FREE_SWING;
			// current_state = DoublePendulumState::COAST_DOWN_TEST;
		}
		break;
	}

	case DoublePendulumState::MONITOR_AS5600: {
		static unsigned long stable_since_ms = 0;

		if (millis() - stable_since_ms > 4000)
			stable_since_ms = millis();

		float threshold = PI + 1;
		float cos_inner = cos(inner_angle_rads);

		if (false)
			stable_since_ms = millis();
		else if (millis() - stable_since_ms >= 3000) {
			LOOP_LOG("PENDULUM IS STABLE AND UPRIGHT");
			// current_state = DoublePendulumState::NN_BALANCING;
			current_state = DoublePendulumState::NN_SWINGUP;
			// current_state = DoublePendulumState::MOTOR_CUVRE_TEST;
			// current_state = DoublePendulumState::BREAKAWAY_TEST;
		}

		break;
	}

	case DoublePendulumState::NN_BALANCING: {

		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;
		float cart_v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		if (cos(inner_angle_rads) < cos(PI / 2.0) || abs(pos_m) > 0.4) {
			current_state = DoublePendulumState::EMERGENCY_BRAKE;
			odrv0.setTorque(0.0f);
			break;
		}

		float force_n = -double_pendulum_policy(-pos_m, -cart_v_mps, inner_angle_rads, dt_s) * 40.0;
		float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[NN] force_n = %.6f,\ttorque_nm = %.6f\n", force_n, torque_nm);

		odrv0.setTorque(torque_nm);

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

		float force_n = -double_pendulum_policy(-pos_m, -cart_v_mps, inner_angle_rads, dt_s) * 40.0;

		float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[NN] force_n = %.6f,\ttorque_nm = %.6f\n", force_n, torque_nm);

		odrv0.setTorque(torque_nm);

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
			current_state = DoublePendulumState::IDLE;
		}

		break;
	}
	case DoublePendulumState::MOTOR_CUVRE_TEST: {
		static float force_n = -30.0f;
		static float torque_nm = force_n * PULLEY_RADIUS_M;

		odrv0.setTorque(torque_nm);

		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		Serial.println(String(dt_s) + "," + String(torque_nm, 6) + "," + String(pos_m, 6) + "," + String(v_mps, 6) +
		               "," + String(inner_angle_rads, 6));

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
	case DoublePendulumState::FREE_SWING: {
		static bool first_pass = true;
		static float inner_angle_prev = 0.0f;

		if (first_pass) {
			inner_angle_prev = inner_angle_rads;
			Serial.println("time_s,pole_cos,pole_sin,pole_angle_rad,pole_vel_radps");
			first_pass = false;
			break;
		}

		float cos_angle = cos(inner_angle_rads);
		float sin_angle = sin(inner_angle_rads);
		float cos_inner_angle_prev = cos(inner_angle_prev);
		float sin_inner_angle_prev = sin(inner_angle_prev);
		float angular_vel_radps =
		    ((sin_angle - sin_inner_angle_prev) * cos_angle - (cos_angle - cos_inner_angle_prev) * sin_angle) / dt_s;

		inner_angle_prev = inner_angle_rads;

		Serial.println(String(dt_s) + "," + String(cos_angle, 6) + "," + String(sin_angle, 6) + "," +
		               String(inner_angle_rads, 6) + "," + String(angular_vel_radps, 6));

		break;
	}

	case DoublePendulumState::EMERGENCY_BRAKE: {
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;
		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;

		int8_t sign = v_mps > 0.0 ? -1 : 1;

		static float force_n = 80.0f * sign;
		static float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[RUNNING] [EMERGENCY_BRAKE] v_mps = %.6f,\t force_n = %.6f", v_mps, force_n);

		if (abs(v_mps) > 0.0 && abs(pos_m) > 0.3) {
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

float double_pendulum_policy(const float cart_pos_m, const float cart_vel_mps, const float angle_rads2,
                             const float dt_s) {
	static bool first_run = true;
	static float angle_prev = 0.0f;

	if (first_run) {
		angle_prev = angle_rads2;
		first_run = false;
		return 0.0f;
	}

	float upright_offset = 0.03;

	float angle_rads = angle_rads2 - upright_offset;

	float cos_angle = cos(angle_rads);
	float sin_angle = sin(angle_rads);
	float cos_angle_prev = cos(angle_prev);
	float sin_angle_prev = sin(angle_prev);
	float angular_vel_radps =
	    ((sin_angle - sin_angle_prev) * cos_angle - (cos_angle - cos_angle_prev) * sin_angle) / dt_s;

	angle_prev = angle_rads;

	float obs[5] = {cart_pos_m, cart_vel_mps, sin_angle, cos_angle, angular_vel_radps};
	scale_observations(obs, 5);

	input->data.f[0] = obs[0];
	input->data.f[1] = obs[1];
	input->data.f[2] = obs[2];
	input->data.f[3] = obs[3];
	input->data.f[4] = obs[4];

	interpreter->Invoke();

	LOOP_LOG("[FUNCTION] dt_s = %.6f, cart_pos_m = %.6f,\tcart_vel_mps = %.6f,\tcos = %.5f,\tsin = "
	         "%.5f,\tpole_vel_radps = %.5f",
	         dt_s, cart_pos_m, cart_vel_mps, cos_angle, sin_angle, angular_vel_radps);

	return output->data.f[0];
}
