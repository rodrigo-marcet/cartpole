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
	init_led();    // visual feedback
	init_serial(); // debug first
	init_i2c();    // bus before devices
	// initEncoder();   // validates AS5600 presence //TODO
	init_can();
	init_odrive();
	// initFSM();
	// initTimers();

	BOOT_LOG("SETUP FINISHED\n");
}

void loop() {
	hfsm();
}
