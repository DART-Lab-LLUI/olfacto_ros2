**olfacto_ros2** is a collection of ROS 2 packages developed for the olfactometer project. These packages facilitate the integration and control of olfactometer devices within ROS 2-based robotic systems.

## Requirements
The packages are developed and tested for the following hardware/software:
- Raspberry Pi 4B
- Ubuntu Server 22.04
- ROS 2 Humble

## Installation

To install the `olfacto_ros2` packages, run the following commands:

```bash
git clone https://github.com/DART-Lab-LLUI/olfacto_ros2.git
cd olfacto_ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
