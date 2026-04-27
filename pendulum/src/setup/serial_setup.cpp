#include "serial_setup.h"
#include "src/config.h"

#include "LED_setup.h"

#include "src/utils/log_macros.h"

void init_serial() {
	Serial.begin(SERIAL_BAUD);
	while (!Serial && millis() < 3000)
		; // wait up to 3s for USB CDC

	if (millis() >= 3000)
		halt_with_led(Color::RED);

	BOOT_LOG("Serial OK");
}
