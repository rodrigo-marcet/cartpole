#pragma once

#include <ODriveCAN.h>
#include <ODriveEsp32Twai.hpp>

#include "src/utils/odrive_types.h"

struct EncoderEstimatesResult {
	bool ok;
	float pos;
	float vel;
};

// Declare as extern — defined once in odrive_setup.cpp
extern ODriveCAN odrv0;
extern ODriveCAN *odrives[];
extern ODriveUserData odrv0_user_data;

EncoderEstimatesResult get_encoder_estimates();
