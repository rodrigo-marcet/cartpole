#include <Arduino.h>
#include <Wire.h>

#include "src/hfsm.h"

#include "src/config.h"
#include "src/setup/serial_setup.h"
#include "src/setup/I2C_setup.h"
#include "src/setup/LED_setup.h"
#include "src/setup/CAN_setup.h"
#include "src/setup/odrive_setup.h"

#include "src/utils/log_macros.h"

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

	BOOT_LOG("\n");
}

void loop() {
	hfsm();
}
/* ----------------- Loop ---------------- */
