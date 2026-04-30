#include "src/utils/odrive.h"

#include "src/utils/log_macros.h"
#include "src/utils/odrive_types.h"

EncoderEstimatesResult get_encoder_estimates() {
	Get_Encoder_Estimates_msg_t fb;
	if (odrv0.request(fb, 10)) { // 100ms timeout
		// LOOP_LOG("pos: %.2f, vel: %.2f", fb.Pos_Estimate, fb.Vel_Estimate);
		return {true, fb.Pos_Estimate, fb.Vel_Estimate};
	}
	return {false, 0.0f, 0.0f};
}
