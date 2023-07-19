# WPG

A graphical interface to test/control a drone inside ROS (Robotic operating system).

The acronymous means "**WayPoint Generator**" as this generates points for the drone to follow during the simulation sending messages on the position_setpoint or velocity_setpoint topics.

Any contribuition is welcome. Just open a PR if you have something you want to improve in the code and I will review when/if I have time.

## Reference articles
Some articles that cited the use of this program:

#### 2021
- [Unmanned Aerial Vehicles Motion Control with Fuzzy Tuning of Cascaded-PID Gains](https://sciprofiles.com/publication/view/7c10a222af668029a1b5a761268e5589) (Journal)
- [Hybrid PID-Fuzzy controller for autonomous UAV stabilization](https://ieeexplore.ieee.org/abstract/document/9529680) (Conference)
#### 2022
- [Fuzzy Gain-Scheduling PID for UAV Position and Altitude Controllers](https://sciprofiles.com/publication/view/73ddf2b8e67dc6924f095988241d2912) (Journal)
#### 2023
- [Sliding Mode Controller Applied to Autonomous UAV Operation in Marine Small Cargo Transport](https://ieeexplore.ieee.org/document/10185965) (Journal)

## Videos

https://www.youtube.com/watch?v=9c_JVZtPlU8

## Notes
This project assumes that you are using PX4, Gazebo, and the version Noetic Ninjemys of ROS1 for Ubuntu 20.04 (Focal) release.

The "how to use" section also explains how to install PX4, gazebo and ROS.

## How to use

WIP

## How to install/setup

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

If catkin_make fails, probably you are using an python environment manager and ROS is stupid when installing the dependencies because he installs on the system-wide python instead, that should fix:

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

Now we have it, but we also need the drone control algorithm to run with this.

[There is](https://github.com/piradata/PX4-Autopilot) a fork of the PX4 Firmware codebase that had the internal controller on the most internal cascade control loop modified to be an SMC controller instead of an PID controller. This inner loop is responsible to control the drone angular rate. 

Lets clone this inside an specific folder.

```
mkdir -p ~/src/
cd ~/src/
git clone https://github.com/piradata/PX4-Autopilot.git Firmware
cd Firmware/
```

After that, to run a simulation on ROS with a drone inside that uses mavlink protocol and topics, just run the following commands

### Running the simulation

```shell
# This one assuming you are building the simulation based on PX4 project and uses gazebo as the simulation environment 
make px4_sitl_default gazebo_iris
```

> **Note**
>
> By default this command will use the `iris.world` world file, but if you want to use an specific word file for testing set it with `PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/empty.world` at the end of the make command.

> **Note**
>
> The model of drone this command runs is the file `iris.sdf.jinja` on folder `/src/Firmware/Tools/sitl_gazebo/models/iris`. You can either modify the model manually or set to another one changing `gazebo_iris` to `gazebo_typhoon_h480`.

> **Note**
>
> For a full explanation of the ways of running this please visit [PX4 Guide - Building PX4](https://docs.px4.io/main/en/dev_setup/building_px4.html). There are also the solution of some of the problem mentioned below on the "Possible problems" section.

#### Possible problems

- It is possible that the compile fails because one of the following packages are missing, so just install them:

```bash
sudo apt install liblzma-dev lzma
pip3 install toml numpy packaging jinja2
sudo apt install libgstreamer1.0-dev
sudo apt install genromfs ninja-build exiftool astyle
sudo apt install libgstreamer-plugins-base1.0-dev
```

- Also, if you get an error about lib lzma, that must means that you are using python from an virtual environment and building from source, if that is the case you may need to rebuild the python install after having libs like liblzma-dev in the system. If you are running python from an virtual env I assume you know what you are doing but just in case the command I use to rebuild is this (using asdf):

```bash
asdf uninstall python 3.10.5
asdf install python 3.10.5
```

- Another problem can can occur is `c++: fatal error: Killed signal terminated program cc1plus`, that means the compiler is using more memory than the system has. To bypass this you can compile single core with `make -j1` instead of just `make` or increase the size of your swap partition.

- And if you have any problems with gstreamer lib as `gst_app_src_push_internal: assertion 'GST_IS_APP_SRC (appsrc)' failed` or `g_object_set: assertion 'G_IS_OBJECT` don't hesitate to do the following:

```bash
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio
```

### Connecting the mavros node

If the compiler ends with success, just open another terminal and run the following command (keep the terminal with make open)

```shell
# connect the ros node to the flight control unit thought mavlink to be able to read/write on mavros topics (also assuming PX4 project)
roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'
```

Also, if this breaks because of `geographicLib exception: File not readable`, you can fix with this:

```bash
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
wget -O - https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh | sudo bash
```

### Running the WPG package

Now open another terminal and run the following command (keep the terminal with make and the terminal with mavros both open)

```shell
# start the graphical simulation
rosrun wpg v6_fuzzy_vel_smc_py3.py
```

If the `PySimpleGUI27` python lib is missing, just install it!

```bash
pip3 install PySimpleGUI27
```

## Tools

On this repository there is also a matlab script `matlab_plotter_mavros_v2.m` that is used to plot the data of the mission based on a BAG file as input. To generate a BAG file that record the simulation data run the following in another terminal while the simulation is running:

`rosbag record -a -O sample_name`

That will generate a `sample_name.BAG` file on the current directory you are in.

Them, edit the file `matlab_plotter_mavros_v2.m` or `matlab_plotter_mavros.m` in the root folder of this project and put the bag file there. 

```matlab
% Controllers = ["SMC_PID_8", "SMC_PID_8_WW", "PID", "PID_WW"];
Controllers = ["sample_name"];
```

Them run the script. A series of plots will be generated regarding the data of the flight.

![PID_flower](https://user-images.githubusercontent.com/16529503/232749541-c9894713-d90e-43ec-a349-a0b29c9750f3.png)

![driv_erro_novo_com_vento](https://user-images.githubusercontent.com/16529503/232749570-2f9de2ee-cd87-4fb8-b074-c21955a4d5fd.png)

![path_complete_four_way](https://user-images.githubusercontent.com/16529503/232749640-30118156-5bcf-41cb-b64e-70bcddadc413.png)

## Screenshots

![image](https://user-images.githubusercontent.com/16529503/232741516-008664d9-df62-415c-b34a-e35700e8c785.png)

![image](https://user-images.githubusercontent.com/16529503/232742277-d9336f5e-b539-48cd-8247-166015bdc6b1.png)

![image](https://user-images.githubusercontent.com/16529503/232742358-de88f516-e510-4f0e-9e54-68877984b0ea.png)

![image](https://user-images.githubusercontent.com/16529503/232742530-44aeb25c-c152-4e86-a2d6-574f22b18c4a.png)

![image](https://user-images.githubusercontent.com/16529503/232742685-3c1e6941-4ad4-4cfb-b9a6-452cab69352e.png)

![image](https://user-images.githubusercontent.com/16529503/232742798-1b5cfc9d-bbf6-42ac-96cb-9907f2a04fbd.png)
