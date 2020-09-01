# grbl_ros ![Rolling CI](https://github.com/flynneva/grbl_ros/workflows/Rolling%20CI/badge.svg)

A simple ROS2 package for interfacing with a [grbl device](https://github.com/gnea/grbl).

Currently supports:
- polling status of grbl device
- sending plain GCODE commands using ROS2 service

## Supported Platforms
**All Tier 1 platforms = Windows 10, Mac OS X, and either Ubuntu 18.04 or Ubuntu 20.04**
OS           | ROS 2 Version | CI Status | 
------------ | ------------- | --------- |
All Tier 1 platforms | [Dashing Diademata](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/) | ![Dashing CI](https://github.com/flynneva/grbl_ros/workflows/Dashing%20CI/badge.svg) | 
All Tier 1 platforms | [Eloquent Elusor](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/) | ![Eloquent CI](https://github.com/flynneva/grbl_ros/workflows/Eloquent%20CI/badge.svg) |
All Tier 1 platforms | [Foxy Fitzroy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/) | ![Foxy CI](https://github.com/flynneva/grbl_ros/workflows/Foxy%20CI/badge.svg) |
All Tier 1 platforms | [Rolling Ridley](https://index.ros.org/doc/ros2/Releases/Release-Rolling-Ridley/) | ![Rolling CI](https://github.com/flynneva/grbl_ros/workflows/Rolling%20CI/badge.svg) |

Dashing and Eloquent target Ubuntu 18.04 while Foxy and Rolling target Ubuntu 20.04.

## Getting started

Quick start:
```bash
# for Ubuntu
sudo apt install ros-<your-distro>-grbl-ros
```

Fork and clone the repository if you'd like to compile it yourself.
Once cloned, you can now build your workspace by running the following from your workspace's root directory:
```bash
colcon build
```

## Testing
Unit tests are run on every PR and every release across every supported platform for each ROS2 release. Refer to [the "actions" tab for this repository](https://github.com/flynneva/grbl_ros/actions?query=workflow%3A%22ROS+2+CI%22) to see these tests yourself.

## TODO
Here is a list of potential ideas/features this package could have. Eventually the plan is to add all of these.
- full grbl command support & descriptions (non-modal commands, motion modes, etc.)
- safety features that grbl already implements to be translated to ROS (door sensor, estop, endstop, etc.)
- full file gcode streaming to grbl device
- publish tf of grbl machine coordinates
- ROS services for jog control
- im sure there are more but I cant think of them right now
