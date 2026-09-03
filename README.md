# Self-Balancing Double and Single Pendulum

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

<p align="center">
  <img src="docs\rig.gif" width="600" alt="[Project] hardware photo">
</p>

## Overview

Arduino/C++ firmware for real-time control of self-balancing single and double inverted pendulums. The firmware interfaces with magnetic encoders and motor controllers to estimate the system state and execute control policies on an ESP32, without requiring a host PC. It is designed as the embedded component of a sim-to-real reinforcement learning pipeline, deploying policies trained in NVIDIA Isaac Lab directly to the physical hardware. The repo with all of the RL build can be seen here.

## How It Works

The firmware runs on an ESP32-S3 and follows a hierarchical finite-state-machine architecture, handling startup calibration, sensor acquisition, and real-time control of the pendulum. Encoder measurements are converted into the system state and fed into the active controller, which commands the BLDC motor through the motor controller to stabilize the pendulum. The repository also includes the hardware configuration and supporting files needed to build and deploy the firmware directly to the embedded system.

## Repository Structure

```text
.
├── pendulum/              # Arduino firmware for the pendulum controller
│   ├── src/
│   │   ├── models/        # Control and ML models
│   │   ├── scripts/       # Firmware scripts and supporting logic
│   │   ├── setup/         # Hardware and system initialization
│   │   ├── states/        # Hierarchical finite-state machine states
│   │   ├── utils/         # Shared utilities and helpers
│   │   ├── config.h       # Hardware and controller configuration
│   │   └── hfsm.*         # Hierarchical finite-state machine
│   └── pendulum.ino       # Arduino application entry point
├── odrvconfig.txt         # Motor controller configuration
└── README.md              # You are here
```


> See [`pendulum/README.md`](pendulum/README.md) for build/firmware/project-structure details.

## Hardware
Links to every component i used for this build.

* Microcontroller: [ESP32-S3](ALIEXPRESS_URL)
* BLDC motor controller: [MKS XDrive Mini](ALIEXPRESS_URL)
* Drive Motor: [5065 BLDC Motor](ALIEXPRESS_URL)
* Shaft encoders: [AS5600 Magnetic Encoder](ALIEXPRESS_URL)
* Others: [CAN Bus Transceiver](ALIEXPRESS_URL), [Slip Ring](ALIEXPRESS_URL)


## Getting Started

[Minimal steps to clone and run/build the top-level project. Link out to the
sub-folder README for the detailed procedure rather than duplicating it.]

```bash
git clone https://github.com/[user]/[repo].git
cd [repo]
[build/run command]
```

## License

Distributed under the [MIT] License. See [`LICENSE`](LICENSE) for details.
