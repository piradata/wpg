#resumo

cd ~/src/Firmware

sudo make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/uneven.world

roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'

rosrun wpg fuzzy_v.py

rosrun rqt_graph rqt_graph

rostopic echo ...


#aliases:
alias cdpx4="cd ~/src/Firmware"
alias makepx4="sudo make no_sim=1 px4_sitl_default gazebo"
alias gazset="source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default"
alias gazinit="roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world"
alias rossim="roslaunch mavros px4.launch fcu_url:='udp://:14550@127.0.0.1:14555'"


# instalação completa
cd ~
mkdir src
cd src
git clone https://github.com/piradata/Firmware.git
cd Firmware/Tools/setup/
source ubuntu.sh
cd ~
mkdir code
cd code
wget https://raw.githubusercontent.com/PX4/Devguide/master/build_scripts/ubuntu_sim_ros_melodic.sh
source ubuntu_sim_ros_melodic.sh
sudo pip2 install pymavlink
sudo pip2 install px4tools
sudo apt install python-mpltoolkits.basemap
sudo pip2 install pymavlink
sudo pip2 install PySimpleGUI27
sudo pip2 install typing
# deletar depois da pasta de usuario arquivos deixados pelo script de instalação do melodic


# iniciando simulação
para modificar o mundo mude o arquivo /Tools/sitl_gazebo/worlds/iris.world
export PX4_HOME_LAT=0.0
export PX4_HOME_LON=0.0
export PX4_HOME_ALT=0.0
export PX4_SIM_SPEED_FACTOR=1

cdpx4
makepx4
cdpx4

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris.world

OU

sudo make px4_sitl_default gazebo_iris PX4_SITL_WORLD:=$(pwd)/Tools/sitl_gazebo/worlds/uneven.world
# conectando ao simulador
rossim

# rodando codigos
rosrun wpg ...

# vendo nós
rosrun rqt_graph rqt_graph

# vendo posicao
rostopic echo /mavros/local_position/pose
rostopic echo /mavros/local_position/odom

# on px4cmd
commander takeoff
commander land

#Modo de controle
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm
rosrun mavros mavcmd land 0 0 0 0

#publish position on ros:
rostopic pub -r 20 /mavros/setpoint_position/local geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: ""}, pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}'

#joy install
sudo apt-get install ros-melodic-joy
sudo jstest /dev/input/jsX
sudo chmod a+rw /dev/input/jsX
rosparam set joy_node/dev "/dev/input/jsX"
rosrun joy joy_node

#controlar drone com teclado
roslaunch mavros_extras teleop.launch teleop_args e
roslaunch mavros_extras teleop.launch

# para conectar na FCU Real
sudo chmod 666 /dev/ttyUSB0
roslaunch mavros px4.launch fcu_url:=/dev/ttyUSB0:57600

# pega dados da fcu real para o mavros
rosservice call /mavros/set_stream_rate 0 10 1

# configura mensagens personalizadas no matlab
folderpath = fullfile('~/catkin_ws','src')
rosgenmsg(folderpath)
addpath('/home/piradata/catkin_ws/src/matlab_gen/msggen')
savepath
REINICIAR MATLAB
rosmsg list

# gazebo help
# https://github.com/ArduPilot/ardupilot_wiki/issues/1001

###################################################
# criar script python ou cpp no ros e ativa-lo


# editar mundo iris para ter um chão decente
/home/guilherme/catkin_ws/src/mavros/test_mavros/launch/px4/iris_empty_world_offboard_ctl.launch
/home/guilherme/src/Firmware/Tools/sitl_gazebo/worlds/iris.world
