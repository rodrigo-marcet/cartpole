#include "LED_setup.h"

#include "Adafruit_NeoPixel.h"

#define LED_PIN 48 // ESP32-S3 has a built-in NeoPixel on GPIO48
#define LED_COUNT 1

static Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void init_led() {
	led.begin();
	led.setBrightness(50); // don't blind yourself
	led.clear();
	led.show();
}

void set_led(Color color) {
	led.setPixelColor(0, (uint32_t)color);
	led.show();
}

void halt_with_led(Color color) {
	while (true) {
		set_led(color);
		delay(300);
		set_led(Color::OFF);
		delay(300);
	}
}
