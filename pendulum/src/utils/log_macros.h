#pragma once

#define BOOT_LOG(fmt, ...)                                                                                             \
	do {                                                                                                               \
		Serial.printf("[BOOT] " fmt "\n", ##__VA_ARGS__);                                                              \
	} while (0)

#define BOOT_ERROR(fmt, ...)                                                                                           \
	do {                                                                                                               \
		BOOT_LOG("[ERROR] " fmt, ##__VA_ARGS__);                                                                       \
	} while (0)
