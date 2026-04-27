#pragma once

#include <ODriveCAN.h>
#include <ODriveEsp32Twai.hpp> // provides pumpEvents(), wrap_can_intf(); requires onCanFrame()

#include "src/config.h"

// Per-ODrive user data (same fields as official example)
struct ODriveUserData {
	Heartbeat_msg_t last_heartbeat;
	bool received_heartbeat = false;
};

// Declare as extern — defined once in odrive_setup.cpp
extern ODriveCAN odrv0;
extern ODriveCAN *odrives[];
extern ODriveUserData odrv0_user_data;

// Called on Heartbeat from ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data);

// Called on encoder feedback from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data);

// Your TWAI adapter expects this hook; forward to all ODriveCAN instances
void onCanFrame(uint32_t id, uint8_t len, const uint8_t *data);

void init_odrive();
