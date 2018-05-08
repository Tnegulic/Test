# Turtlebot 3 - Segway

Kratki uvod

## 1. Slaganje

Praćenje Turtlebot3 quickstart uputa, slaganje modela segway [ROBOTIS TurtleBot3 Segway](https://cad.onshape.com/documents/f369a265c003df3767a37472/w/42e27bfc98b5e204f5a483fe/e/5b993b12303647b7aa5484f2)

## 2. PC setup

*Instalacija Ubuntu 16.04 operacijskog sustava - * [Ubuntu_16.04](https://www.ubuntu.com/download/desktop)

*Instalacija ROS Kinetic
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

*Instalacija potrebnih paketa
```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

*Konfiguracija mreže
```
$ nano ~/.bashrc
```
dodavanje na kraju datoteke (eg. HOST_IP = 192.168.0.101)
```
export ROS_MASTER_URI=http://HOST_IP:11311
export ROS_HOSTNAME=HOST_IP
```
```
$ source ~/.bashrc
```
## RaspberryPI setup