#include <Arduino.h>
#include <Wire.h>

#include "src/hfsm.h"

#include "src/config.h"

#include "src/setup/serial_setup.h"
#include "src/setup/I2C_setup.h"
#include "src/setup/LED_setup.h"
#include "src/setup/CAN_setup.h"
#include "src/setup/odrive_setup.h"
#include "src/setup/tflite_setup.h"

#include "src/utils/log_macros.h"

/* ----------------- Setup ---------------- */

void setup() {
	init_led();    // visual feedback
	init_serial(); // debug first
	init_i2c();    // bus before devices
	// initEncoder();   // validates AS5600 presence //TODO
	init_can();
	init_odrive();
	init_tflite();

	BOOT_LOG("SETUP FINISHED\n");
}

void loop() {
	hfsm();
}

//   // --- Warm-up run ---
//   float dummy[4] = {0.1f, 0.05f, 0.0f, 0.0f};
//   for (int i = 0; i < 4; i++) input->data.f[i] = dummy[i];
//   interpreter->Invoke();

//   // --- Benchmark: 1000 runs ---
//   const int N = 1000;
//   uint32_t t0 = micros();
//   for (int i = 0; i < N; i++) {
//     for (int j = 0; j < 4; j++) input->data.f[j] = dummy[j];
//     interpreter->Invoke();
//   }
//   uint32_t t1 = micros();

//   float avg_us = (float)(t1 - t0) / N;
//   Serial.printf("Avg inference: %.1f us\n", avg_us);
//   Serial.printf("Max freq:      %.0f Hz\n", 1e6f / avg_us);
//   Serial.printf("Action output: %.6f\n", output->data.f[0]);
// }

// void loop() {}
