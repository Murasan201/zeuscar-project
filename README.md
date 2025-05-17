# ZeusCar Project

A modular three-tier robot control platform leveraging ROS 2 and Arduino to enable seamless communication between a host PC, a Raspberry Pi, and an Arduino motor controller.

**Prerequisite**: ROS 2 (Humble Hawksbill or later) must already be installed and sourced on your system.

## Directory Structure

```
zeuscar-project/
├── README.md        ← Overview of the entire project (this file)
├── arduino/         ← Arduino firmware source and flashing instructions
│   └── README.md    ← Detailed Arduino setup and usage guide
├── ros2/            ← ROS 2 workspace on Raspberry Pi
│   └── README.md    ← Build, run, and test instructions for ROS 2
├── pc_control/      ← ROS 2 Publisher node code on the host PC
│   └── README.md    ← Dependencies and launch instructions for the Publisher
└── .gitignore       ← Git ignore settings for build artifacts and temp files
```

## Component Overview

### Host PC
- **OS**: Ubuntu Desktop 22.04
- **Role**: Run the ROS 2 Publisher node to send control commands to the robot
- **Connection**: Communicates over the same LAN network

### Raspberry Pi 4
- **Model**: Raspberry Pi 4B (or 5)
- **OS**: Ubuntu Server 22.04
- **Role**: Run the ROS 2 Subscriber node to receive commands and forward them via serial to the Arduino
- **Connection**: LAN connection to the host PC; powered by SunFounder PiPower

### Arduino
- **Model**: Arduino Uno R3
- **Role**: Receive serial commands from the Raspberry Pi and drive the motors accordingly
- **Connection**: USB serial link to the Raspberry Pi

## Author

- **Author**: Murasan
- **Website**: https://murasan-net.com/

## License

This project is released under the **MIT License**. See the `LICENSE` file in the repository root for full terms.
