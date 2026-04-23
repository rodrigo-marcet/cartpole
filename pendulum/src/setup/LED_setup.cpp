#include "LED_setup.h"

#include "Adafruit_NeoPixel.h"

#define LED_PIN 48 // ESP32-S3 has a built-in NeoPixel on GPIO48
#define LED_COUNT 1

static Adafruit_NeoPixel led(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void initLED() {
	led.begin();
	led.setBrightness(50); // don't blind yourself
	led.clear();
	led.show();
}

void setLED(uint32_t color) {
	led.setPixelColor(0, color);
	led.show();
}

void haltWithLED(uint32_t color) {
	while (true) {
		setLED(color);
		delay(300);
		setLED(0);
		delay(300);
	}
}
