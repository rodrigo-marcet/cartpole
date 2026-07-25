#pragma once

#include "src/utils/types.h"
#include "src/utils/hfsm_types.h"
#include "src/utils/odrive_types.h"

enum class DoublePendulumState : uint8_t {
	ENABLE_CONTROL_LOOP_CONTROL,
	WAIT_FOR_CONTROL_LOOP_CONTROL,
	ENABLE_TYPE_CONTROL,

	MONITOR_AS5600,
	NN_BALANCING,
	NN_SWINGUP,

	BREAKAWAY_TEST,
	MOTOR_CUVRE_TEST,
	COAST_DOWN_TEST,
	IDLE,
	COLLECT_DATA,
	FREE_SWING,

	EMERGENCY_BRAKE,
	DONE = 254,
	ERROR = 255,
};

SequenceStatus double_pendulum(DoublePendulumState &current_state, const ODriveCalibrationResult &rail_limits,
                               const EncoderEstimatesResult &fb, const float inner_encoder_rads);

float double_pendulum_policy(const float cart_pos_m, const float cart_vel_mps, const float angle_rads,
                             const float dt_s);
