#include "can_setup.h"

#include <ESP32-TWAI-CAN.hpp>

#include "src/config.h"

static bool setupCan() {
	const auto kbps = CAN_BAUDRATE / 1000;
	// You can omit setPins if begin() accepts pins; kept explicit for clarity
	ESP32Can.setPins(CAN_TX_PIN, CAN_RX_PIN);
	ESP32Can.setRxQueueSize(16);
	ESP32Can.setTxQueueSize(16);
	return ESP32Can.begin(ESP32Can.convertSpeed(kbps), CAN_TX_PIN, CAN_RX_PIN);
}

void initCAN() {
	if (!setupCan()) {
		Serial.println("CAN failed to initialize: reset required");
		while (true) {
			delay(50);
		}
	}
}
