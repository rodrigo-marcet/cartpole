#pragma once

#include <ODriveCAN.h>
#include <ODriveEsp32Twai.hpp> // provides pumpEvents(), wrap_can_intf(); requires onCanFrame()

// Per-ODrive user data (same fields as official example)
struct ODriveUserData {
	Heartbeat_msg_t last_heartbeat;
	bool received_heartbeat = false;
};

enum class Direction : int8_t { NEGATIVE = -1, POSITIVE = 1 };
