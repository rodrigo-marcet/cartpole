#include "src/states/running/main_sequence.h"

#include <cmath>

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"
#include "src/utils/tflite.h"
#include "src/config.h"

// Input normalisation (running mean / variance from SKRL preprocessor)
float input_mean[] = {0.03160070f, -0.04923343f, 0.00269673f, 0.97471118f, 0.13676924f};
float input_var[] = {0.00872619f, 0.38674623f, 0.02406406f, 0.02587135f, 6.36017227f};

constexpr int POSITION_PID_DECIMATION = 5;

SequenceStatus main_sequence(MainSequenceState &current_state, const ODriveCalibrationResult &rail_limits,
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
	case MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL: {
		smoothed_action = 0.0f;
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
		static unsigned long stable_since_ms = 0;

		if (millis() - stable_since_ms > 4000)
			stable_since_ms = millis();

		float threshold = 0.1;
		float cos_inner = cos(inner_angle_rads);

		// if ((PI - threshold >= inner_angle_rads) || (inner_angle_rads >= PI + threshold))
		if (cos_inner < 0.8)
			stable_since_ms = millis();
		else if (millis() - stable_since_ms >= 3000) {
			LOOP_LOG("PENDULUM IS STABLE AND UPRIGHT");
			current_state = MainSequenceState::NEURAL_NETWORK;
		}

		break;
	}

	case MainSequenceState::PENDULUM_PID: {
		static int call_count = 0;
		static float goal_deviation = 0.0f;

		if (call_count % POSITION_PID_DECIMATION == 0) {
			goal_deviation = position_pid(rail_limits.midpoint, fb.pos, dt_s * POSITION_PID_DECIMATION);
		}
		call_count++;

		SequenceStatus status = pendulum_pid(inner_angle_rads, dt_s, PI - goal_deviation);

		break;
	}
	case MainSequenceState::NEURAL_NETWORK: {

		if (cos(inner_angle_rads) < cos(PI / 2)) {
			odrv0.setTorque(0.0f);
			current_state = MainSequenceState::ERROR;

			LOOP_LOG("[NN] pole fell down at angle: %.6f", inner_angle_rads);
			break;
		}

		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;

		float cart_v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;

		float force_n = -neural_network(-pos_m, -cart_v_mps, inner_angle_rads, dt_s) * 40.0;

		float torque_nm = force_n * PULLEY_RADIUS_M;

		LOOP_LOG("[NN] force_n = %.6f,\ttorque_nm = %.6f\n", force_n, torque_nm);

		odrv0.setTorque(torque_nm);

		break;
	}

	case MainSequenceState::RAMP_UP_SPEED: {
		float rps_for_1_mps = 1.0f / PULLEY_CIRCUMFERENCE_M; // v_mps = rps * 2 * PI * radius
		float velocity_rps = -rps_for_1_mps * 1.0f;
		odrv0.setVelocity(velocity_rps * 1.1);

		if (abs(fb.vel) > abs(velocity_rps + 0.1f)) {
			LOOP_LOG("[RUNNING] [RAMP UP] velocity matches what we expected");
			current_state = MainSequenceState::IDLE;
		}
		break;
	}

	case MainSequenceState::IDLE: {
		if (!odrv0.setControllerMode(ODriveControlMode::CONTROL_MODE_TORQUE_CONTROL,
		                             ODriveInputMode::INPUT_MODE_PASSTHROUGH)) {
			LOOP_ERROR("[RUNNING] [MAIN] Switching to position control was not possible");
			current_state = MainSequenceState::ERROR;
		} else {
			LOOP_LOG("[RUNNING] [MAIN] IDLE");
			current_state = MainSequenceState::DONE;
		}
		break;
	}
	case MainSequenceState::COLLECT_DATA: {
		float v_mps = fb.vel * PULLEY_CIRCUMFERENCE_M;
		float pos_m = (fb.pos - rail_limits.midpoint) * PULLEY_CIRCUMFERENCE_M;

		Serial.println(String(dt_s) + "," + String(pos_m, 6) + "," + String(v_mps, 6));

		if (abs(v_mps) < abs(0.05)) {
			current_state = MainSequenceState::DONE;
		}
		break;
	}

	case MainSequenceState::STATIC_FRICTION: {
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
			current_state = MainSequenceState::IDLE;
		}

		break;
	}

	case MainSequenceState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		current_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;

		LOOP_LOG("[RUNNING] [MAIN] DONE");
		// return SequenceStatus::DONE;
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

float position_pid(const float midpoint_m, const float current_pos_m, const float dt_s) {

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = midpoint_m - current_pos_m;

	float p_gain = 0.044;
	float p_term = error * p_gain;

	if (first_run) {
		prev_error = error;
		first_run = false;
	}

	float d_gain = 0.012; // best 0.011
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

	LOOP_LOG("goal = %.2f,\tpos = %.2f,\tdeviation = %.2f,\tp_term = %.2f,\td_term = %.2f,\ti_term = %.2f", midpoint_m,
	         current_pos_m, deviation, p_term, d_term, i_term);

	return deviation;
}

SequenceStatus pendulum_pid(const float angle_rads, const float dt_s, const float goal_angle_rads) {

	float upright_offset = 0.05;

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = (goal_angle_rads + upright_offset) - angle_rads;

	float p_gain = 0.8;
	float p_term = error * p_gain;

	if (first_run) {
		prev_error = error;
		first_run = false;
	}

	float d_gain = 0.03;
	float d_term = (error - prev_error) / dt_s * d_gain;

	prev_error = error;

	float max_torque_nm = 5.0f;

	static float integral = 0.0f;

	integral += error * dt_s;

	float i_gain = 0.3;
	float max_integral = max_torque_nm / i_gain;
	integral = std::clamp(integral, -max_integral, max_integral);

	float i_term = integral * i_gain;
	i_term = std::clamp(i_term, -max_torque_nm, max_torque_nm);

	float torque_nm = p_term + d_term + i_term;
	torque_nm = std::clamp(torque_nm, -max_torque_nm, max_torque_nm);

	// LOOP_LOG("goal = %.6f,\tangle = %.6f,\ttorque_nm = %.6f,\tp_term = %.6f,\td_term = %.6f,\ti_term = %.6f",
	// goal_angle_rads,
	//          angle_rads, torque_nm, p_term, d_term, i_term);
	LOOP_LOG("goal = %.6f,\tangle = %.6f,\ttorque_nm = %.6f", goal_angle_rads, angle_rads, torque_nm);

	odrv0.setTorque(torque_nm);

	return SequenceStatus::RUNNING;
}

float neural_network(const float cart_pos_m, const float cart_vel_mps, const float angle_rads, const float dt_s) {
	static bool first_run = true;
	static float angle_prev = 0.0f;

	if (first_run) {
		angle_prev = angle_rads;
		first_run = false;
		return 0.0f; // don't invoke the network yet
	}

	float cos_angle = cos(angle_rads);
	float sin_angle = sin(angle_rads);
	float cos_angle_prev = cos(angle_prev);
	float sin_angle_prev = sin(angle_prev);
	float angular_vel_radps =
	    ((sin_angle - sin_angle_prev) * cos_angle - (cos_angle - cos_angle_prev) * sin_angle) / dt_s;

	angle_prev = angle_rads;

	// if(abs(angular_vel_radps) > 10.0f)
	// 	return 0.0f;
	float obs[5] = {cart_pos_m, cart_vel_mps, sin_angle, cos_angle, angular_vel_radps};
	scale_observations(obs);

	input->data.f[0] = obs[0];
	input->data.f[1] = obs[1];
	input->data.f[2] = obs[2];
	input->data.f[3] = obs[3];
	input->data.f[4] = obs[4];

	interpreter->Invoke();

	LOOP_LOG("[FUNCTION] cart_pos_m = %.6f,\tcart_vel_mps = %.6f,\tcos = %.5f,\tsin = %.5f,\tpole_vel_radps = %.5f",
	         cart_pos_m, cart_vel_mps, cos_angle, sin_angle, angular_vel_radps);
	// LOOP_LOG("cos_angle = %.6f,\tsin_angle = %.6f,\tangle = %.6f,\tangular_vel_radpsocity = %.6f",
	// 	cos_angle, sin_angle, angle_rads, angular_vel_radps);
	// Serial.println(String(dt_s, 3) + "," + String(cos_angle, 6) + "," + String(sin_angle, 6) + "," + String(angle,
	// 6)+ "," + String(angular_vel_radps, 6));

	return output->data.f[0];
	// return 0.0;
}

void scale_observations(float obs[5]) {
	for (int i = 0; i < 5; i++) {
		obs[i] = (obs[i] - input_mean[i]) / (sqrtf(input_var[i]) + 1e-8f);
	}
}
