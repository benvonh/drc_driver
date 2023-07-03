# drc_driver

ROS2 driver for the Droid Racing Challenge.

## Config

A `config.hpp` file is provided under `include/drc_driver/` which contains the pin numbers for speed and steering on
Raspbery Pi 4, in addition to the minimum and maximum pulsewidth for the servo steering and ESC.

Default configuration:
- Speed pin: 12
- Steering pin: 13
- Min pulsewidth: 10ms
- Max pulsewidth: 20ms

## Usage

The following creates a node `/driver` which subscribes to two topics:
- `/speed_command`
- `/steer_command`
Both use the `std_msgs/msg/Int32` interface to represent a percentage of the pulsewidth from minimum to maximum PWM.
If `-1` is provided, the pin is temporarily deactivated until further updated. Otherwise, the topic callbacks will
expect an integer between 0 and 100 in which it converts to a pulsewidth and applies it on the respective pin.

Run driver in normal mode.
```sh
ros2 run driver
```

Calibrate ESC before subscribing to command topics
```sh
ros2 run driver --ros-args --param calibrate:=1
```
