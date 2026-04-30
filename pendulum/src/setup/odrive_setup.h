#pragma once

#include "src/utils/odrive_types.h"

#include "src/config.h"

// Called on Heartbeat from ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data);

// Called on encoder feedback from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data);

// Your TWAI adapter expects this hook; forward to all ODriveCAN instances
void onCanFrame(uint32_t id, uint8_t len, const uint8_t *data);

void init_odrive();
