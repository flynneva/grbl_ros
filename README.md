# grbl_ros ![ROS2 CI](https://github.com/flynneva/grbl_ros/workflows/ROS2%20CI/badge.svg)

A simple ROS2 package for interfacing with a [grbl device](https://github.com/gnea/grbl).

Currently supports:
- polling status of grbl device
- sending plain GCODE commands using ROS2 service

## Supported Platforms
**All Tier 1 platforms = Windows 10, Mac OS X, and either Ubuntu 18.04 or Ubuntu 20.04**
OS           | ROS 2 Version |
------------ | ------------- |
All Tier 1 platforms | [Dashing Diademata](https://index.ros.org/doc/ros2/Releases/Release-Dashing-Diademata/) | 
All Tier 1 platforms | [Eloquent Elusor](https://index.ros.org/doc/ros2/Releases/Release-Eloquent-Elusor/) |
All Tier 1 platforms | [Foxy Fitzroy](https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/) |
All Tier 1 platforms | [Rolling Ridley](https://index.ros.org/doc/ros2/Releases/Release-Rolling-Ridley/) |

Dashing and Eloquent target Ubuntu 18.04 while Foxy and Rolling target Ubuntu 20.04.

## How to use
The `grbl_ros` package exposes two major functionalities to ROS: pose tracking of the GRBL device and sending commands to the GRBL device.

This package publishes a transform (aka `tf`) to the most recent machine coordinates received from the GRBL device. This allows for other ROS nodes to "know" where the machine is which can help enable many different functions.

Secondly this package runs two ROS2 actions: `send_gcode_cmd` and `send_gcode_file`. Both actions do what they say they do and enable the user to monitor the status of these actions as they happen.

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

Once built, just make sure you've sourced your overlay (`source /path/to/workspace/install/setup.bash`) and you should be able to start up the `grbl_ros` node:

```
ros2 run grbl_ros grbl_node --ros-args --params-file /path/to/workspace/src/grbl_ros/config/cnc001.yaml
```

Before you can command your GRBL device to move, most likely you'll need to unlock it using the `$X` command:
```
ros2 action send_goal /cnc_001/send_gcode_cmd grbl_msgs/action/SendGcodeCmd '{command: $X}'
```

You should see `Goal finished with status: SUCCEEDED` and a print-out in the `grbl_ros` terminal that the GRBL device is now unlocked.

From here, you can send your device whatever GCODE commands you would like. Here are a few common ones to get you started:
```
ros2 action send_goal /cnc_001/send_gcode_cmd grbl_msgs/action/SendGcodeCmd '{command: X1.0}'
# TBD, recommend more if you'd like more to be listed here!
```


## Testing
Unit tests are run on every PR and every release across every supported platform for each ROS2 release. Refer to [the "actions" tab for this repository](https://github.com/flynneva/grbl_ros/actions?query=workflow%3A%22ROS+2+CI%22) to see these tests yourself.

## Contributing
Are you using `grbl_ros`? Do you like what functionality it provides but want it to do something more? PLEASE reach out!!! I want to hear from you! Make an issue, PR or smoke signal to get my attention as I would love to hear from you!
