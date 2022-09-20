# wpg

This project is an graphical interface to test/control a drone inside ROS (Robotic operating system)

## How to use

Clone it inside catkin workspace and build the project with `catkin make`

After that, to run a simulation on ROS with a drone inside that uses mavlink protocol and topics, just run the folloing commands

```shell
# this one issuming you are building the simulation based on PX4 project and uses gazebo as the simulation environment
make px4_sitl_default gazebo_iris

# connect the ros node to the flight control unit thought mavlink to be able to read/write on mavros topics (also assuming PX4 project)
roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'

# start the graphical simulation
rosrun wpg v6_fuzzy_vel_smc_py3.py
```

OBS: this project assumes that you are using PX4, Gazebo, and the version Noetic Ninjemys of ROS1 for Ubuntu 20.04 (Focal) release.
