#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class MainSequenceState : uint8_t {
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	ENABLE_TYPE_CONTROL,
	SINUSOIDAL_POS,
	POSITION_PID,
	MONITOR_AS5600,
	PENDULUM_PID,
	NEURAL_NETWORK,

	DONE = 254,
	ERROR = 255,
};

SequenceStatus main_sequence(MainSequenceState &current_state, const ODriveCalibrationResult &rail_limits,
                             const EncoderEstimatesResult &fb, const float inner_encoder_rads);

float position_pid(const float midpoint, const float current_pos, const float dt_s);

SequenceStatus pendulum_pid(const float as5600_rads, const float dt_s, const float goal_angle);

float neural_network(const float cart_pos, const float cart_vel, const float as5600_rads, const float dt_s);
