#pragma once

#include <ODriveCAN.h>
#include <ODriveEsp32Twai.hpp>

struct EncoderEstimatesResult {
	bool ok;
	float pos;
	float vel;
};

EncoderEstimatesResult get_encoder_estimates();
