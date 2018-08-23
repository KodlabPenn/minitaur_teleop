# Minitaur remote control with ROS

This package is based on the [Ghost Robotics SDK](http://ghostrobotics.gitlab.io/SDK/). See the `CommandOverEthernet` example in the SDK to properly setup the ROS bridge over Ethernet.

## Usage
Once the ROS bridge has been setup and running, launch `minitaur_teleop.launch` to control the robot. 

## Setting up the remote
Here we assume that an Xbox360 controller is used. To set this controller up:
1. Install the appropriate packages:
```
$ sudo apt-get install jstest* xboxdrv
```

2. Ensure that `xpad` is not getting loaded by opening `/etc/modprobe.d/blacklist.conf` and adding `blacklist xpad` at the end of the file.
3. Run `xboxdrv` with your controller connected:
```
$ xboxdrv --silent
```

4. If necessary, configure your controller axes and buttons:
```
$ jstest-gtk
```