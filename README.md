# Ikerlan

![distro](https://img.shields.io/badge/Ubuntu%2022-Jammy%20Jellyfish-green)
![distro](https://img.shields.io/badge/ROS2-Humble-blue)
[![main](https://github.com/IntelligentRoboticsLabs/ikerlan/actions/workflows/master.yaml/badge.svg?branch=main)](https://github.com/IntelligentRoboticsLabs/ikerlan/actions/workflows/master.yaml)

# Installation on your own computer

Prepare your thirparty repos:
```bash
sudo apt update
sudo apt install python3-vcstool python3-pip python3-rosdep python3-colcon-common-extensions -y
cd <ros2-workspace>/src/
vcs import < ikerlan/thirdparty.repos
```

*Please make sure that this last command has not failed. If this happens, run it again.*

### Install libusb, libftdi & libuvc
```bash
sudo apt install libusb-1.0-0-dev libftdi1-dev libuvc-dev
```

### Install udev rules from astra camera, kobuki and rplidar
When you connect a piece of hardware to your pc, it assigns `/dev/ttyUSB*` to it. This will not have the necessary read/write permissions, so we will not be able to use it correctly. The solution is to set up some udev rules that creates a symlink with another name (example: `/dev/ttyUSB0` -> `/dev/kobuki`) and grants it the necessary permissions.
```bash
cd <workspace-ros2>
sudo cp src/ThirdParty/ros_astra_camera/astra_camera/scripts/56-orbbec-usb.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/rplidar_ros/scripts/rplidar.rules /etc/udev/rules.d/
sudo cp src/ThirdParty/kobuki_ftdi/60-kobuki.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Move xtion calibration
Some cameras need a calibration file where they indicate, for example, their resolution, name, etc...
```bash
mkdir -p ~/.ros/camera_info
cp <ros2-workspace>/src/ThirdParty/openni2_camera/openni2_camera/rgb_PS1080_PrimeSense.yaml ~/.ros/camera_info
```

### Building project
```bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

# About

This is a project made by the [Intelligent Robotics Lab], a research group from the [Universidad Rey Juan Carlos].
Copyright &copy; 2023.

Contributors:

* [Alberto García]
* [Francisco Martín]
* [José Miguel Guerrero]
* [Juan Carlos Manzanares]
* [Juan Diego Peña]
* [Rodrigo Pérez]

## License

[![CC BY-SA 4.0][cc-by-sa-shield]][cc-by-sa]

This work is licensed under a
[Creative Commons Attribution-ShareAlike 4.0 International License][cc-by-sa].

[![CC BY-SA 4.0][cc-by-sa-image]][cc-by-sa]

[cc-by-sa]: http://creativecommons.org/licenses/by-sa/4.0/
[cc-by-sa-image]: https://licensebuttons.net/l/by-sa/4.0/88x31.png
[cc-by-sa-shield]: https://img.shields.io/badge/License-CC%20BY--SA%204.0-lightgrey.svg

[Universidad Rey Juan Carlos]: https://www.urjc.es/
[Intelligent Robotics Lab]: https://intelligentroboticslab.gsyc.urjc.es/
[José Miguel Guerrero]: https://sites.google.com/view/jmguerrero
[Juan Carlos Manzanares]: https://github.com/Juancams
[Francisco Martín]: https://github.com/fmrico
[Alberto García]: https://sites.google.com/view/aaggj
[Juan Diego Peña]: https://sites.google.com/view/juandpenan
[Rodrigo Pérez]: https://sites.google.com/view/rodperex
