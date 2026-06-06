#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class MainSequenceState : uint8_t {
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	ENABLE_TYPE_CONTROL,
	MOVE_TO_LIMIT,
	RAMP_UP_SPEED,
	IDLE,
	COLLECT_DATA,
	STATIC_FRICTION,

	MONITOR_AS5600,
	PENDULUM_PID,
	NEURAL_NETWORK,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus main_sequence(MainSequenceState &current_state, const ODriveCalibrationResult &rail_limits,
                             const EncoderEstimatesResult &fb, const float inner_encoder_rads);

float position_pid(const float midpoint, const float current_pos, const float dt_s);

SequenceStatus pendulum_pid(const float angle_rads, const float dt_s, const float goal_angle_rads);

float neural_network(const float cart_pos_m, const float cart_vel_mps, const float angle_rads, const float dt_s);

void scale_observations(float obs[5]);
