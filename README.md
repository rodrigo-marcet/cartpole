# Pendulum

Double inverted pendulum on a cart, built and controlled from scratch.
This repo documents the full build — hardware, firmware, and ML controller.

## Hardware

- ESP32-S3
- AS5600 magnetic encoder
- BLDC motor + ODESC 4.2
- GT2 belt drive, 2020 V-slot rail

## Firmware

Written in C++ for Arduino CLI. Hierarchical FSM architecture:
- **Calibration** — zeros the AS5600 encoder from the hang position at boot
- **Running** — reads encoder, runs controller

## Compile & Upload

```bash
cd pendulum
arduino-cli compile --fqbn esp32:esp32:esp32s3 .
arduino-cli upload --fqbn esp32:esp32:esp32s3 -p COM3 .
arduino-cli monitor -p COM3 -c baudrate=115200
```

## Dependencies

- Arduino ESP32 core
- Wire (built-in)
