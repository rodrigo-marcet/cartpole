#include <Arduino.h>
#include <Wire.h>

#include "src/hfsm.h"

#include "src/config.h"
#include "src/setup/serial_setup.h"
#include "src/setup/I2C_setup.h"
#include "src/setup/LED_setup.h"
#include "src/setup/CAN_setup.h"
#include "src/setup/odrive_setup.h"

// void loop() {
// 	hfsm();
// }
/* ----------------- Setup ---------------- */

void setup() {
	initLED();    // visual feedback
	initSerial(); // debug first
	initI2C();    // bus before devices
	// initEncoder();   // validates AS5600 presence //TODO
	initCAN();
	initODrive();
	// initFSM();
	// initTimers();
}

/* ----------------- Loop ---------------- */

void loop() {
	// Required to handle incoming feedback frames
	pumpEvents(ESP32Can);

	// Official example motion: sine position with velocity feedforward
	const float SINE_PERIOD_S = 2.0f;
	float t = 0.001f * millis();
	float phase = t * (TWO_PI / SINE_PERIOD_S);

	odrv0.setPosition(sinf(phase),                           // position (turns)
	                  cosf(phase) * (TWO_PI / SINE_PERIOD_S) // velocity feedforward
	);

	// Print position & velocity for Serial Plotter
	if (odrv0_user_data.received_feedback) {
		auto fb = odrv0_user_data.last_feedback;
		odrv0_user_data.received_feedback = false;
		Serial.print("odrv0-pos:");
		Serial.print(fb.Pos_Estimate);
		Serial.print(",");
		Serial.print("odrv0-vel:");
		Serial.println(fb.Vel_Estimate);
	}

	// Keep loop light; TWAI RX is already polled via pumpEvents()
	delay(2);
}
