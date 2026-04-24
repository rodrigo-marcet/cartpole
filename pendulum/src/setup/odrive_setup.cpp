#include "odrive_setup.h"

#include "LED_setup.h"
#include "src/utils/log_macros.h"

ODriveCAN odrv0(wrap_can_intf(ESP32Can), ODRV0_NODE_ID);
ODriveCAN *odrives[] = {&odrv0};
ODriveUserData odrv0_user_data;

// Called on Heartbeat from ODrive
void onHeartbeat(Heartbeat_msg_t &msg, void *user_data) {
	auto *d = static_cast<ODriveUserData *>(user_data);
	d->last_heartbeat = msg;
	d->received_heartbeat = true;
}

// Your TWAI adapter expects this hook; forward to all ODriveCAN instances
void onCanFrame(uint32_t id, uint8_t len, const uint8_t *data) {
	Serial.println(id, HEX); // <-- add this
	for (auto *odrive : odrives) {
		odrive->onReceive(id, len, data);
	}
}

void initODrive() {
	// Register ODrive callbacks
	odrv0.onStatus(onHeartbeat, &odrv0_user_data);

	// Wait for ODrive heartbeat (pump events; add tiny yield)
	BOOT_LOG("Waiting for ODrive heartbeat...");
	uint32_t t = millis();
	while (!odrv0_user_data.received_heartbeat) {
		pumpEvents(ESP32Can);
		delay(1);
		if (millis() - t > 5000) {
			BOOT_ERROR("ODrive heartbeat timeout");
			haltWithLED(Color::YELLOW);
		}
	}
	BOOT_LOG("ODrive found");

	// Request bus voltage/current (1s timeout)
	BOOT_LOG("Attempting to read bus voltage and current");
	Get_Bus_Voltage_Current_msg_t vbus;
	if (!odrv0.request(vbus, 1000)) {
		BOOT_ERROR("vbus request failed");
		haltWithLED(Color::YELLOW);
	}
	BOOT_LOG("DC voltage [V]: ");
	Serial.println(vbus.Bus_Voltage);
	BOOT_LOG("DC current [A]: ");
	Serial.println(vbus.Bus_Current);

	// Enter CLOSED_LOOP_CONTROL with periodic event pumping (mirrors official flow)
	BOOT_LOG("Enabling CLOSED_LOOP_CONTROL...");
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
			BOOT_ERROR("closed loop transition timeout");
			haltWithLED(Color::YELLOW);
		}
	}

	BOOT_LOG("ODrive Ok");
}
