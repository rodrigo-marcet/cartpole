#include "can_setup.h"

#include <ESP32-TWAI-CAN.hpp>

#include "src/config.h"
#include "src/setup/LED_setup.h"

#include "src/utils/log_macros.h"

static bool setupCan() {
	const auto kbps = CAN_BAUDRATE / 1000;
	// You can omit setPins if begin() accepts pins; kept explicit for clarity
	ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
	ESP32Can.setRxQueueSize(16);
	ESP32Can.setTxQueueSize(16);
	return ESP32Can.begin(ESP32Can.convertSpeed(kbps), CAN_TX_PIN, CAN_RX_PIN);
}

void init_can() {
	if (!setupCan()) {
		BOOT_ERROR("CAN failed to initialize: reset required");
		halt_with_led(Color::ORANGE);
	}
	BOOT_LOG("CAN Ok");
}
