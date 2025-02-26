**olfacto_ros2** is a collection of ROS 2 packages and Arduino code developed for the olfactometer project. 

## Requirements
The ROS packages are developed and tested for the following hardware/software:
- Raspberry Pi 4B
- Ubuntu Server 22.04
- ROS 2 Humble

The arduino code:
- Valve control: Arduino Mega 2560 Rev3
- Sensor publish: Arduino Nano

## Installation

To install the `olfacto_ros2` packages, run the following commands:

```bash
git clone https://github.com/DART-Lab-LLUI/olfacto_ros2.git
cd olfacto_ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build
```
