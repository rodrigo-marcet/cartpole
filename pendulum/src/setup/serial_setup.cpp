#include "serial_setup.h"
#include "src/config.h"

void initSerial() {
	Serial.begin(SERIAL_BAUD);
	while (!Serial && millis() < 3000)
		; // wait up to 3s for USB CDC
	Serial.println("[BOOT] Serial OK");
}
