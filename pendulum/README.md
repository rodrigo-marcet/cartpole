# Project Structure

This document describes how the firmware is organized. For the project overview, hardware, demo media, and general introduction, see the [top-level README](../README.md).

## Directory Layout

```text
pendulum/
├── src/
│   ├── models/
│   │   ├── single_pendulum/      # Control policies for the single pendulum
│   │   └── double_pendulum/      # Control policies for the double pendulum
│   ├── scripts/
│   │   └── read_serial.py        # Serial data logging utility
│   ├── setup/
│   │   ├── I2C_setup.*           # I2C bus initialization
│   │   ├── LED_setup.*           # Status LED initialization
│   │   ├── can_setup.*           # CAN interface initialization
│   │   ├── odrive_setup.*        # Motor controller initialization
│   │   ├── serial_setup.*        # Serial interface initialization
│   │   └── tflite_setup.*        # TensorFlow Lite initialization
│   ├── states/
│   │   ├── calibration/          # Hardware calibration state
│   │   └── running/              # Runtime control states
│   ├── utils/
│   │   ├── as5600.*              # AS5600 encoder interface
│   │   ├── odrive.*              # Motor controller communication
│   │   ├── tflite.*              # TensorFlow Lite inference interface
│   │   ├── *_types.h             # Shared data structures and types
│   │   └── log_macros.h          # Logging utilities
│   ├── config.h                  # Hardware and controller configuration
│   ├── hfsm.h                    # Hierarchical FSM interface
│   └── hfsm.cpp                  # Hierarchical FSM implementation
├── pendulum.ino                  # Arduino application entry point
└── README.md                     # This document
```

## Module Breakdown

### `models/`

Contains the control policies deployed to the ESP32. Models are separated into `single_pendulum` and `double_pendulum` directories, with different balancing and swing-up policies available for each system.

The model files contain the embedded policy data required for inference on the microcontroller rather than loading a model from external storage at runtime.

### `setup/`

Contains initialization and hardware-specific setup code. Each subsystem has its own setup module, keeping peripheral initialization separate from the control logic.

The setup layer currently covers:

* I2C communication for the AS5600 encoders
* CAN communication
* Motor controller initialization
* Serial communication
* Status LED
* TensorFlow Lite runtime initialization

### `states/`

Implements the runtime state machine.

The top-level states are:

* `calibration/`: performs the hardware calibration sequence, including encoder and motor-controller setup.
* `running/`: contains the runtime control states for the single and double pendulum.

The running state is further divided by the controlled system, allowing the single- and double-pendulum controllers to share the same state-machine infrastructure while maintaining separate control logic.

### `utils/`

Contains low-level interfaces and shared utilities used throughout the firmware.

This includes the AS5600 encoder driver, motor-controller communication layer, TensorFlow Lite interface, shared types, and logging macros.

### `config.h`

Central configuration point for hardware-specific parameters, controller configuration, and compile-time constants.

### `hfsm.h / hfsm.cpp`

Implements the hierarchical finite-state machine that coordinates the firmware's high-level execution flow.

The HFSM provides the common state-management layer used by the calibration and running states.

### `pendulum.ino`

Arduino entry point. It initializes the system and drives the top-level firmware execution.

## Architecture

The firmware is organized around a hierarchical finite-state machine. At startup, the system initializes its hardware interfaces and enters the calibration state. Once calibration is complete, execution transitions into the running state, where sensor measurements are acquired, converted into the controller's observation space, and passed to the appropriate control policy.

The resulting action is sent to the motor controller through the communication layer, closing the real-time control loop.

## Dependencies

* **Arduino ESP32 core**: ESP32-S3 board support and hardware APIs.
* **Wire**: I2C communication with the AS5600 magnetic encoders.
* **TensorFlow Lite Micro**: embedded inference of the deployed control policies.
* **CAN interface**: communication between the ESP32-S3 and motor controller.

## Build & Upload

The firmware is built using Arduino CLI with the ESP32-S3 target:

```bash
cd pendulum

arduino-cli compile --fqbn esp32:esp32:esp32s3 .

arduino-cli upload --fqbn esp32:esp32:esp32s3 -p COM3 .

arduino-cli monitor -p COM3 -c baudrate=2000000
```

Replace `COM3` with the serial port assigned to the ESP32-S3.

## Notes

The firmware is intended for the specific hardware configuration documented in the top-level repository. In particular, the deployed policies depend on the observation and action interfaces used during training, so changing encoder scaling, coordinate conventions, timing, or actuator configuration may require retraining or adapting the corresponding policy.

The repository currently contains both single- and double-pendulum policies, allowing the simpler single-pendulum controller to be used independently while developing and validating the control stack.
