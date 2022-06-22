Code to control the zed gimbal, using pi3hat and moteus
===============================================
### About

zed_gimbal is responsible for connecting the rover network's LCM interface with the zed gimbal system's motor controller, the moteus. Note that this code has never actually been used on the rover.

### Topics - Publisher

**Zed Gimbal Position Data [Publisher]** \
Messages: [ZedGimbalPosition.msg](https://github.com/umrover/mrover-ros/blob/main/msg/ZedGimbalPosition.msg) "zed_gimbal_data" \
Publisher: zed_gimbal \
Subscribers: nav and gui

### Topics - Subscriber

**Zed Gimbal Position Cmd [Subscriber]** \
Messages: [ZedGimbalPosition.msg](https://github.com/umrover/mrover-ros/blob/main/msg/ZedGimbalPosition.msg) "zed_gimbal_cmd" \
Publisher: nav \
Subscribers: zed_gimbal

### Usage

SSH into the raspberrypi.

Enable multicast by running `sudo ifconfig eth0 multicast` and `sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth0`.
Replace eth0 with whatever the connection is if that does not work.

### Common Errors

When first integrating the zed gimbal motor with the moteus controller, there were issues with wobbling and jittering. There is a thread on the MJBots discord server called "2022-01-22 new motor configuration" that goes over some of these problems and its solutions. The following will summarize a few of these errors.

For the 2022 zed gimbal motor controller, voltage_mode_control had to be set to 1 (changing it from current mode control to voltage mode control) because our motor had a range of currents of 0-1A while the moteus sense resistors were trying to sense currents between 0-100A.

Also, the moteus should be setup so that its PID values are tuned. The simplest way to do this is through tview. One way is to forward the gui onto the computer (one can use VcXsrv or Xming). Otherwise, the easiest way is to connect the raspberry pi to a monitor with an HDMI cable.


### Notes

Reference.md for moteus is [here](https://github.com/mjbots/moteus/blob/main/docs/reference.md).

## TODO
- [ ] Finish this readme
- [ ] Change from left-handed coordinate system to right
- [ ] Figure how to ssh into pi through linux system
- [ ] Enable multicast be default
- [ ] There needs to be a much better setup document

## TODO - ROS Migration
- [ ] Test if any of this stuff still works
- [ ] Check if async stuff is broken
- [ ] Need to somehow add a launch file
- [ ] Make sure moteus and pi3hat libraries are recognized... will need to eventually get it into the rosdistro repository. Read more [here](http://docs.ros.org/en/jade/api/catkin/html/howto/format1/python_module_dependencies.html)
