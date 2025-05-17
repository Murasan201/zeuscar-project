# Arduino Firmware: raspi-ctrl-v_2_00.ino

This directory contains the Arduino sketch for the ZeusCar platform. The `raspi-ctrl-v_2_00.ino` sketch receives text commands over serial and drives four motors using software PWM.

## Prerequisites

- **Arduino IDE** (version 1.8.0 or later)
- **Board**: Arduino Uno R3
- **Library**: SoftPWM (install via Library Manager)
- **Connection**: USB cable between Arduino Uno and Raspberry Pi

## Setup

1. **Launch Arduino IDE**
2. **Install SoftPWM library**
   - Go to **Sketch > Include Library > Manage Libraries...**
   - Search for **SoftPWM** and click **Install**
3. **Select board and port**
   - **Tools > Board**: Choose **Arduino Uno**
   - **Tools > Port**: Select the correct COM/TTY port

## Uploading the Sketch

1. **Open `raspi-ctrl-v_2_00.ino`** in Arduino IDE.
2. Click **Verify** (✔️) to compile.
3. Click **Upload** (→) to flash the firmware to the Arduino Uno.
4. Monitor **Serial Monitor** at **9600 baud** to see incoming commands and debug output.

## Usage

- On the Raspberry Pi, run the ROS 2 Subscriber node which sends ASCII commands terminated by `\n`.
- The Arduino will execute motor movements based on the received command strings.

---

For additional details, refer to the main project documentation in the repository root.
