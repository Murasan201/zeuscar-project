# PC Control Node: publisher.py

This directory contains the ROS?2 Publisher node for the host PC, which reads user input and sends control commands to the Raspberry Pi via ROS?2. The Raspberry?Pi subscriber node will receive these commands and forward them to the Arduino motor controller.

**Prerequisite**: ROS?2 (Humble Hawksbill or later) must already be installed and sourced on your system.

## File Structure

```
pc_control/
├── publisher.py      ← ROS?2 Publisher node source
└── README.md         ← (this file)
```

## Dependencies

* **Python**: 3.8 or later
* **ROS?2**: Humble Hawksbill (with `rclpy`, `std_msgs` installed)

You may want to use a virtual environment:

```bash
cd pc_control
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt   # if you create a requirements file
```

## How It Works

* The script reads numeric input from the user (`0`?`10`) corresponding to predefined commands:

  * `0`: FORWARD
  * `1`: BACKWARD
  * `2`: LEFT
  * …
  * `10`: STOP
* It maps the number to a command string and publishes it on the ROS?2 topic `topic`.
* The Raspberry?Pi subscriber node listens on `topic` and acts accordingly.

## Running the Publisher

1. **Source ROS?2**

   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. **Run the node**

   ```bash
   python3 publisher.py
   ```
3. **Enter commands** in the prompt (e.g., `0` to move forward) or type `exit` to quit.

```text
Enter command number (0=FORWARD, 1=BACKWARD, etc.) or 'exit':
```
