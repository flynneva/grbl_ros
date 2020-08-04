# grbl_ros ![ROS 2 CI](https://github.com/flynneva/grbl_ros/workflows/ROS%202%20CI/badge.svg)

A simple ROS2 package for interfacing with a GRBL device.

Currently supports:
- polling status of GRBL device
- sending plain GCODE commands using ROS2 service

## Supported Platforms
**All = Windows 10, Mac OS X**

OS           | ROS 2 Version | 
------------ | ------------- | 
All, Ubuntu 18.04 | Dashing Diademata | 
All, Ubuntu 18.04 | Eloquent Elusor | 
All, Ubuntu 20.04 | Foxy Fitzroy | 

## Getting started

First make sure you have one of the supported ROS2 distro's installed and sourced.

Go ahead and clone this repo into your workspace `src` directory.
```bash
# if you use ssh
git clone git@github.com:flynneva/grbl_ros.git
# if you dont
git clone https://github.com/flynneva/grbl_ros.git
```

Once cloned, you can now build your workspace by running the following from your workspace's root directory:
```bash
colcon build
```

## TODO
Here is a list of potential ideas/features this package could have. Eventually the plan is to add all of these.
- full file gcode streaming to GRBL device
- transform to GRBL device
- im sure there are more but I cant think of them right now
