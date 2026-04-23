#include "odrive_setup.h"

#include "LED_setup.h"

ODriveCAN odrv0(wrap_can_intf(ESP32Can), ODRV0_NODE_ID);
ODriveCAN *odrives[] = {&odrv0};
ODriveUserData odrv0_user_data;

// Called on Heartbeat from ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data) {
	auto *d = static_cast<ODriveUserData *>(user_data);
	d->last_heartbeat = msg;
	d->received_heartbeat = true;
}

// Called on encoder feedback from ODrive
void onFeedback(Get_Encoder_Estimates_msg_t &msg, void *user_data) {
	auto *d = static_cast<ODriveUserData *>(user_data);
	d->last_feedback = msg;
	d->received_feedback = true;
}

// Your TWAI adapter expects this hook; forward to all ODriveCAN instances
void onCanFrame(uint32_t id, uint8_t len, const uint8_t *data) {
	for (auto *odrive : odrives) {
		odrive->onReceive(id, len, data);
	}
}

void initODrive() {
	// Register ODrive callbacks
	odrv0.onFeedback(onFeedback, &odrv0_user_data);
	odrv0.onStatus(onHeartbeat, &odrv0_user_data);

	// Wait for ODrive heartbeat (pump events; add tiny yield)
	Serial.println("[BOOT] Waiting for ODrive heartbeat...");
	uint32_t t = millis();
	while (!odrv0_user_data.received_heartbeat) {
		pumpEvents(ESP32Can);
		delay(1);
		if (millis() - t > 5000) {
			Serial.println("[BOOT] ERROR: ODrive heartbeat timeout");
			haltWithLED(Color::YELLOW);
		}
	}
	Serial.println("[BOOT] ODrive found");

	// Request bus voltage/current (1s timeout)
	Serial.println("[BOOT] Attempting to read bus voltage and current");
	Get_Bus_Voltage_Current_msg_t vbus;
	if (!odrv0.request(vbus, 1000)) {
		Serial.println("[BOOT] ERROR: vbus request failed");
		haltWithLED(Color::YELLOW);
	}
	Serial.print("[BOOT] DC voltage [V]: ");
	Serial.println(vbus.Bus_Voltage);
	Serial.print("[BOOT] DC current [A]: ");
	Serial.println(vbus.Bus_Current);

	// Enter CLOSED_LOOP_CONTROL with periodic event pumping (mirrors official flow)
	Serial.println("[BOOT] Enabling CLOSED_LOOP_CONTROL...");
	while (odrv0_user_data.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
		odrv0.clearErrors();
		delay(1);
		odrv0.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

		// Pump events for ~150ms to ensure reliable state transition even on busy bus
		for (int i = 0; i < 15; ++i) {
			delay(10);
			pumpEvents(ESP32Can);
		}
		if (millis() - t > 10000) {
			Serial.println("[BOOT] ERROR: closed loop transition timeout");
			haltWithLED(Color::YELLOW);
		}
	}

	Serial.println("[BOOT] ODrive running");
}
