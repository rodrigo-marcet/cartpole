#pragma once

#include <ODriveCAN.h>
#include <ODriveEsp32Twai.hpp>

#include "src/utils/odrive_types.h"

// Declare as extern — defined once in odrive_setup.cpp
extern ODriveCAN odrv0;
extern ODriveCAN *odrives[];
extern ODriveUserData odrv0_user_data;

EncoderEstimatesResult get_encoder_estimates();

bool move_to_position(float position, float vel_limit = 1.0f,
                      std::pair<float, float> trap_limits = std::make_pair(0.0f, 0.0f));
