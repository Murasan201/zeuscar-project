# ROS2 Workspace: ZeusCar Robot Package

This directory contains the ROS 2 workspace for the ZeusCar project, responsible for self-localization, sensor drivers, and communication between the PC control node and the Arduino motor controller.

## Prerequisites

- **Operating System**: Ubuntu 20.04 or later (tested on Ubuntu 22.04)
- **ROS 2 Distro**: Humble Hawksbill (ensure `ros-humble-*` packages are installed)
- **Python**: 3.8 or later (for the Python-based subscriber node)
- **Dependencies**: Install missing dependencies with `rosdep`:
  ```bash
  cd ros2
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
  ```

## Setup and Build

1. **Clone the repository** (if not already done):
   ```bash
   cd ~
   git clone https://github.com/Murasan201/zeuscar-project.git
   ```
2. **Navigate to the ROS2 workspace**:
   ```bash
   cd ~/zeuscar-project/ros2
   ```
3. **Build the workspace**:
   ```bash
   colcon build --symlink-install
   ```

## Environment Setup

Before running any nodes, source the workspace setup file:

```bash
source install/setup.bash
```

To auto-source on each new shell, add to your `~/.bashrc`:

```bash
echo "source ~/zeuscar-project/ros2/install/setup.bash" >> ~/.bashrc
```

## Running the Subscriber Node

The `subscriber_node` listens for control commands published by the PC control node and forwards instructions to the Arduino-based motor controller.

```bash
ros2 run zeuscar_robot_package subscriber_node
```

## Code Reference

Implementation of the subscriber node can be found here:

- [subscriber.py](src/zeuscar_robot_package/zeuscar_robot_package/subscriber.py)

## Typical Workflow

1. **Start ROS 2 Core** (if not already running):
   ```bash
   ros2 daemon start
   ```
2. **Build & source** the workspace.
3. **Launch sensor drivers**, e.g.,
   ```bash
   ros2 launch sllidar_ros2 sllidar_launch.py
   ```
4. **Run the subscriber node**:
   ```bash
   ros2 run zeuscar_robot_package subscriber_node
   ```
5. **On the PC**, start the high-level control node in parallel.

## License

This project is released under the **MIT License**. See the root directory `LICENSE` for details.
