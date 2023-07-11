# drc_driver

ROS2 driver for Droid Racing Challenge.

## Config

A `config.hpp` file is provided under `include/drc_driver/` which contains the pin numbers for speed and steering on
the Raspbery Pi 4, in addition to the minimum and maximum pulsewidth.

Default configuration:
- Speed pin: 12
- Steering pin: 13
- Min pulsewidth: 10ms
- Max pulsewidth: 20ms

## Usage

**Calibration**

Calibrates the ESC connected to the speed pin before driving. Ensure the droid is lifted and the battery connection is
easily reachable. Follow the steps printed on the screen. Required only once when the ESC is first turned on.
```sh
ros2 run drc_driver driver --ros-args --param calibrate:=1
```

**Normal Mode**

Initialises the `/driver` node and subscribes to two topics:
- `/speed_command`
- `/steer_command`

Both use the `std_msgs/msg/Int32` interface to represent a percentage of the pulsewidth from minimum to maximum PWM.
If `-1` is published, the pin is temporarily deactivated until further updated. Otherwise, the topic callback will
expect an integer between 0 and 100 in which it converts to a pulsewidth and applies it on the respective pin.
```sh
ros2 run drc_driver driver
```

Manual Mode
---
Reads keypresses in the terminal to control the speed and steering. Does not create subscriptions. The controls are
WASD for standard movement. The speed and steering are only updated upon keypress and will hold that value until a new
keypress is issued or SIGINT is sent.
```sh
ros2 run drc_driver driver --ros-args --param manual:=1
```
