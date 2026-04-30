#include "src/utils/odrive.h"

#include "src/utils/log_macros.h"
#include "src/utils/odrive_types.h"

constexpr float STATIC_VELOCITY_DEVIATION = 0.1f;
constexpr float POSITION_DEVIATION = 0.1f;

EncoderEstimatesResult get_encoder_estimates() {
	Get_Encoder_Estimates_msg_t fb;
	if (odrv0.request(fb, 10)) { // 100ms timeout
		// LOOP_LOG("pos: %.2f, vel: %.2f", fb.Pos_Estimate, fb.Vel_Estimate);
		return {true, fb.Pos_Estimate, fb.Vel_Estimate};
	}
	return {false, 0.0f, 0.0f};
}

bool move_to_position(float position, float vel_limit, std::pair<float, float> trap_limits) {
	odrv0.setTrapezoidalVelLimit(vel_limit);
	odrv0.setTrapezoidalAccelLimits(trap_limits.first, trap_limits.second);
	odrv0.setPosition(position, 0.0f);

	EncoderEstimatesResult res = get_encoder_estimates();
	// LOOP_LOG("pos: %.2f, vel: %.2f", res.pos, res.vel);
	if (fabsf(res.vel) < STATIC_VELOCITY_DEVIATION && fabsf(res.pos - position) <= POSITION_DEVIATION) {
		return true;
	}

	return false;
}
