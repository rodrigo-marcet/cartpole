#include "serial_setup.h"
#include "src/config.h"

#include "LED_setup.h"

void initSerial() {
	Serial.begin(SERIAL_BAUD);
	while (!Serial && millis() < 3000)
		; // wait up to 3s for USB CDC

	if (millis() >= 3000)
		haltWithLED(Color::RED);

	Serial.println("[BOOT] Serial OK");
}
