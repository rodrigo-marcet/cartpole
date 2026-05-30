#include "src/states/running/main_sequence.h"

#include <cmath>

#include "src/utils/odrive.h"
#include "src/utils/odrive_types.h"
#include "src/utils/as5600.h"
#include "src/utils/log_macros.h"
#include "src/utils/tflite.h"

constexpr int POSITION_PID_DECIMATION = 5;
const float SCALER_MEAN[5] = {-0.06093934178352356, -0.053457941859960556, 0.005235779099166393, 0.9642264246940613,
                              0.09907779842615128};
const float SCALER_STD[5] = {0.009108875878155231, 0.2797383666038513, 0.03008274920284748, 0.040163472294807434,
                             6.940155029296875};

SequenceStatus main_sequence(MainSequenceState &current_state, const ODriveCalibrationResult &rail_limits,
                             const EncoderEstimatesResult &fb, const float inner_angle) {

	static unsigned long last_sample_time = 0;
	unsigned long t = micros();
	unsigned long dt = t - last_sample_time;
	if (dt < 10'000)
		return SequenceStatus::RUNNING;

	float dt_s = dt / 1'000'000.0;

	last_sample_time = t;

	static unsigned long closed_loop_timeout = 0;

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

	case MainSequenceState::RAMP_UP_SPEED: {
		static bool started = false;
		static float elapsed = 0.0f;

		if (!started) {
			// odrv0.setTorque(-0.04f);  // fixed step, tune magnitude
			started = true;
		}

		elapsed += dt_s;
		float v_ms = fb.vel * 2 * PI * 0.01;
		float pos_meters = (fb.pos - rail_limits.midpoint) * 2.0f * PI * 0.01f;

		LOOP_LOG("[NN] angle = %.6f,\tposition = %.6f\n", inner_angle, pos_meters);

		// Serial.println(String(elapsed, 6) + "," + String(v_ms, 6));

		if (abs(v_ms) > 1.5f) {
			odrv0.setTorque(0.0f);
			started = false;
			elapsed = 0.0f;
			current_state = MainSequenceState::IDLE;
		}

		break;
	}

	// case MainSequenceState::RAMP_UP_SPEED: {
	// 	static float torque = 0.0f;
	// 	static float elapsed = 0.0f;

	// 	elapsed += dt_s;
	// 	torque = 0.005f * elapsed; // linear ramp: 0.005 Nm/s, tune this

	// 	odrv0.setTorque(torque);
	// 	float v_ms = fb.vel * 2 * PI * 0.01;

	// 	Serial.println(String(dt_s) + ","
	// 		+ String(torque, 6) + ","
	// 		+ String(v_ms, 6));

	// 	if (abs(v_ms) > 0.5f) {
	// 		odrv0.setTorque(0.0f);
	// 		torque = 0.0f;
	// 		elapsed = 0.0f;
	// 		current_state = MainSequenceState::IDLE;
	// 	}

	// 	// float velocity_rps = -15.91549762 * 1.0f;
	// 	// odrv0.setVelocity(velocity_rps * 1.1);
	// 	// // if (fb.pos >= rail_limits.midpoint) {
	// 	// if (abs(fb.vel) > abs(velocity_rps + 0.1f)) {
	// 	// 	LOOP_LOG("[RUNNING] [RAMP UP] velocity matches what we expected");
	// 	// 	current_state = MainSequenceState::IDLE;
	// 	// }
	// 	break;
	// }
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
		float v_ms = fb.vel * 2 * PI * 0.01;
		float pos_meters = (fb.pos - rail_limits.midpoint) * 2.0f * PI * 0.01f;

		Serial.println(String(dt_s) + "," + String(pos_meters, 6) + "," + String(v_ms, 6));

		if (abs(v_ms) < abs(0.05)) {
			current_state = MainSequenceState::DONE;
		}
		break;
	}

	case MainSequenceState::MONITOR_AS5600: {

		static unsigned long stable_since = 0;

		if (millis() - stable_since > 4000)
			stable_since = millis();

		float threshold = 0.1;
		float cos_inner = cos(inner_angle);

		// if ((PI - threshold >= inner_angle) || (inner_angle >= PI + threshold))
		if (cos_inner < 0.8)
			stable_since = millis();
		else if (millis() - stable_since >= 3000) {
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

		SequenceStatus status = pendulum_pid(inner_angle, dt_s, PI - goal_deviation);

		break;
	}
	case MainSequenceState::NEURAL_NETWORK: {
		// auto t_pump = micros();
		float pos_meters = (fb.pos - rail_limits.midpoint) * 2.0f * PI * 0.01f;
		// float cos_angle = cos(inner_angle);
		// float sin_angle = sin(inner_angle);
		float v_ms = fb.vel * 2 * PI * 0.01;

		float force = -neural_network(-pos_meters, -v_ms, inner_angle, dt_s) * 40.0f;
		const float PULLEY_RADIUS = 0.01f;
		float torque = force * PULLEY_RADIUS;

		LOOP_LOG("[NN] force = %.6f,\ttorque = %.6f\n", force, torque);

		odrv0.setTorque(torque);

		break;
	}

	case MainSequenceState::DONE: {
		odrv0.setState(ODriveAxisState::AXIS_STATE_IDLE);
		// current_state = MainSequenceState::ENABLE_CONTROL_LOOP_CONTROL;

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

float position_pid(const float midpoint, const float current_pos, const float dt_s) {

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = midpoint - current_pos;

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

	LOOP_LOG("goal = %.2f,\tpos = %.2f,\tdeviation = %.2f,\tp_term = %.2f,\td_term = %.2f,\ti_term = %.2f", midpoint,
	         current_pos, deviation, p_term, d_term, i_term);

	return deviation;
}

SequenceStatus pendulum_pid(const float angle, const float dt_s, const float goal_angle) {

	float upright_offset = 0.05;

	static bool first_run = true;
	static float prev_error = 0.0f;

	float error = (goal_angle + upright_offset) - angle;

	float p_gain = 0.8;
	float p_term = error * p_gain;

	if (first_run) {
		prev_error = error;
		first_run = false;
	}

	float d_gain = 0.03;
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

	// LOOP_LOG("goal = %.6f,\tangle = %.6f,\ttorque = %.6f,\tp_term = %.6f,\td_term = %.6f,\ti_term = %.6f",
	// goal_angle,
	//          angle, torque, p_term, d_term, i_term);
	LOOP_LOG("goal = %.6f,\tangle = %.6f,\ttorque = %.6f", goal_angle, angle, torque);

	odrv0.setTorque(torque);

	return SequenceStatus::RUNNING;
}

float neural_network(const float cart_pos, const float cart_vel, const float angle, const float dt_s) {
	static bool first_run = true;
	static float angle_prev = 0.0f;

	if (first_run) {
		angle_prev = angle;
		first_run = false;
		return 0.0f; // don't invoke the network yet
	}

	// float angular_vel = (angle - angle_prev) / dt_s;

	float cos_angle = cos(angle);
	float sin_angle = sin(angle);
	float cos_angle_prev = cos(angle_prev);
	float sin_angle_prev = sin(angle_prev);
	float angular_vel = ((sin_angle - sin_angle_prev) * cos_angle - (cos_angle - cos_angle_prev) * sin_angle) / dt_s;

	angle_prev = angle;

	// if(abs(angular_vel) > 10.0f)
	// 	return 0.0f;
	float obs[5] = {cart_pos, cart_vel, sin_angle, cos_angle, angular_vel};
	scale_observations(obs);

	input->data.f[0] = obs[0];
	input->data.f[1] = obs[1];
	input->data.f[2] = obs[2];
	input->data.f[3] = obs[3];
	input->data.f[4] = obs[4];

	interpreter->Invoke();

	LOOP_LOG("[FUNCTION] cart_pos = %.6f,\tcart_vel = %.6f,\tpole_rot = %.6f,\tpole_vel = %.6f", cart_pos, cart_vel,
	         angle, angular_vel);
	// LOOP_LOG("cos_angle = %.6f,\tsin_angle = %.6f,\tangle = %.6f,\toutput = %.6f",
	// 	cos_angle, sin_angle, angle, output->data.f[0]);

	// return std::clamp(output->data.f[0], -40.0f, 40.0f);

	return output->data.f[0];
}

void scale_observations(float obs[5]) {
	for (int i = 0; i < 5; i++) {
		obs[i] = (obs[i] - SCALER_MEAN[i]) / (SCALER_STD[i] + 1e-8f);
	}
}
