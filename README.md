# WPG

A graphical interface to test/control a drone inside ROS (Robotic operating system).

The acronymous mean "**W**ay**P**oing **G**enerator" as this generates points for the drone to follow during the simulation sending messages on the position_setpoint or velocity_setpoint topics.

## Notes
This project assumes that you are using PX4, Gazebo, and the version Noetic Ninjemys of ROS1 for Ubuntu 20.04 (Focal) release.

## How to use

First we need to have ubuntu installed on a appropriated version. For that install ubuntu Focal Fossa so that we can install ROS Noetic.

To confirm that we have ubuntu on the correct version run `cat /etc/os-release` and the output should be something like that:
```
NAME="Ubuntu"
VERSION="20.04.5 LTS (Focal Fossa)"
ID=ubuntu
ID_LIKE=debian
PRETTY_NAME="Ubuntu 20.04.5 LTS"
VERSION_ID="20.04"
HOME_URL="https://www.ubuntu.com/"
SUPPORT_URL="https://help.ubuntu.com/"
BUG_REPORT_URL="https://bugs.launchpad.net/ubuntu/"
PRIVACY_POLICY_URL="https://www.ubuntu.com/legal/terms-and-policies/privacy-policy"
VERSION_CODENAME=focal
UBUNTU_CODENAME=focal
```

After that follow the [Noetic installation tutorial](http://wiki.ros.org/noetic/Installation/Ubuntu) step by step (make sure to install the desktop-full version).

Now we need to configure an catkin workspace to build the packages that will test the simulation.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

If catkin_make fails, probably you are using an python environment manager and ROS is stupid when installing the dependencies, that should fix:

```
pip3 install rosdep rosinstall rosinstall-generator wstool wheel empy catkin_pkg
```

Also lets just now change the setup file called on zshrc or bashrc that was referenced on the installation step to `source ~/catkin_ws/devel/setup.zsh` (or setup.bash if you don't use zsh)

Now lets clone the repository inside the src folder and build the project with `catkin make`

```
cd ~/catkin_ws/src/
git clone https://github.com/piradata/wpg.git wpg
cd ~/catkin_ws/
catkin_make
```

Its possible that some other packages are missing when running `catkin_make`, if so the following commands may help:
```
sudo apt install ros-noetic-geographic-msgs
sudo apt install ros-noetic-libmavconn
sudo apt install ros-noetic-mavros
```

Now we have it, but we also need the drone control algo to run this. There is a fork of the PX4 Firmware codebase that had the internal controller on the most internal cascade control loop to be an SMC controller instead of an PID controller. This inner loop is responsible to control the drone angular rate. Lets clone this inside an specific folder also:

```
mkdir -p ~/src/
cd ~/src/
git clone https://github.com/piradata/PX4-Autopilot.git Firmware
cd Firmware/
```

After that, to run a simulation on ROS with a drone inside that uses mavlink protocol and topics, just run the following commands

```shell
# this one assuming you are building the simulation based on PX4 project and uses gazebo as the simulation environment
make px4_sitl_default gazebo_iris
```

**It is possible that the compile fails because one of the following packages are missing:**

```bash
sudo apt install liblzma-dev lzma
pip3 install toml numpy packaging jinja2
sudo apt install libgstreamer1.0-dev
sudo apt install genromfs ninja-build exiftool astyle
```

Also, if you get an error about lib lzma, that must means that you are using python from an virtual environment and building from source, if that is the case you may need to rebuild the python install after having libs like liblzma-dev in the system. If you are running python from an virtual env I assume you know what you are doing but just in case the command I use to rebuild is this (using asdf):
```
asdf uninstall python 3.10.5
asdf install python 3.10.5
```

If the compiler ends with success, just end continue

```shell
# connect the ros node to the flight control unit thought mavlink to be able to read/write on mavros topics (also assuming PX4 project)
roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'

# start the graphical simulation
rosrun wpg v6_fuzzy_vel_smc_py3.py
```
