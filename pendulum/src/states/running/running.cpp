#include "running.h"

#include <Arduino.h>
#include <Wire.h>

#include "../../utils/as5600.h"
#include "src/setup/odrive_setup.h"
#include "src/utils/log_macros.h"

// SequenceState running_sequence() {
// 	double rads = as5600_read_rads();
// 	double c = cos(rads);

// 	Serial.print(get_as5600_offset());
// 	Serial.print("         ");
// 	Serial.print(c, 6);
// 	Serial.print("\n");

// 	return SequenceState::Running;
// }

SequenceState running_sequence() {
	delay(10);
	pumpEvents(ESP32Can);

	// Official example motion: sine position with velocity feedforward
	const float SINE_PERIOD_S = 2.0f;
	float t = 0.001f * millis();
	float phase = t * (TWO_PI / SINE_PERIOD_S);

	odrv0.setPosition(sinf(phase),                           // position (turns)
	                  cosf(phase) * (TWO_PI / SINE_PERIOD_S) // velocity feedforward
	);

	// Print position & velocity for Serial Plotter
	Get_Encoder_Estimates_msg_t fb;
	if (odrv0.request(fb, 100)) { // 100ms timeout
		LOOP_LOG("pos: %.2f", fb.Pos_Estimate);
		LOOP_LOG("vel: %.2f", fb.Vel_Estimate);
	}

	return SequenceState::Running;
}
