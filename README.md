# ros-pibot

A simple 2 wheel differential drive robot powered by ROS, Arduino &amp; Raspberry Pi 3.

> Tested on Raspbian GNU/Linux 10 (buster), ROS Noetic Installation.

## ROS Installation

> `ROS Noetic` would be compiled from source!

### Prerequistes
Add the Noetic repo list:
```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu buster main" > /etc/apt/sources.list.d/ros-noetic.list'
```
Add the ROS key:
```bash
$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```
Update & check whether the Debian package index is up-to-date: 
```bash
$ sudo apt update
$ sudo apt upgrade
```
Prepare (and install) the dependencies:
```bash
$ sudo apt-get install -y python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential cmake
```
Initialise `rosdep`:
```bash
$ sudo rosdep init
$ rosdep update
```

### Compilation
Create a catkin_ws to compile ROS source.
```bash
$ mkdir -p ~/ros_catkin_ws
$ cd ~/ros_catkin_ws
```
`wstool` would be used to fetch the core packages so we can build them. We would be using the `ROS-Comm` variant plus the navigation packages. (This does not come with GUI tools like rviz and rqt)
```bash
$ rosinstall_generator ros_comm  navigation --rosdistro noetic --deps --wet-only --tar > noetic-ros_comm-nav-wet.rosinstall
$ wstool init src noetic-ros_comm-nav-wet.rosinstall
```
After that, resolve all required dependencies.
```bash
$ cd ~/ros_catkin_ws
$ rosdep install -y --from-paths src --ignore-src --rosdistro noetic -r --os=debian:buster
```
Finally, we'll be building the workspace with `catkin_make_isolated`.
```bash
$ sudo src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/noetic -j2 -DPYTHON_EXECUTABLE=/usr/bin/python3
```
> -j parameter specifies the number of processor(s) to be used, it works fine for -j2, but above that the system could get stuck while compiling as it runs out of memory.

## References
- [ROS Wiki](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Kinetic%20on%20the%20Raspberry%20Pi)
- [VarHowTo](https://varhowto.com/install-ros-noetic-raspberry-pi-4/)


