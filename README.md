# Turtlebot 3 - Segway

Kratki uvod

## 1. Slaganje

* Praćenje Turtlebot3 quickstart uputa, slaganje modela segway [ROBOTIS TurtleBot3 Segway](https://cad.onshape.com/documents/f369a265c003df3767a37472/w/42e27bfc98b5e204f5a483fe/e/5b993b12303647b7aa5484f2)

## 2. PC setup

* Instalacija Ubuntu 16.04 operacijskog sustava - > [Ubuntu_16.04](https://www.ubuntu.com/download/desktop)

* Instalacija ROS Kinetic
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic.sh && chmod 755 ./install_ros_kinetic.sh && bash ./install_ros_kinetic.sh
```

* Instalacija potrebnih paketa
```
$ sudo apt-get install ros-kinetic-joy ros-kinetic-teleop-twist-joy ros-kinetic-teleop-twist-keyboard ros-kinetic-laser-proc ros-kinetic-rgbd-launch ros-kinetic-depthimage-to-laserscan ros-kinetic-rosserial-arduino ros-kinetic-rosserial-python ros-kinetic-rosserial-server ros-kinetic-rosserial-client ros-kinetic-rosserial-msgs ros-kinetic-amcl ros-kinetic-map-server ros-kinetic-move-base ros-kinetic-urdf ros-kinetic-xacro ros-kinetic-compressed-image-transport ros-kinetic-rqt-image-view ros-kinetic-gmapping ros-kinetic-navigation ros-kinetic-interactive-markers
$ cd ~/catkin_ws/src/
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ cd ~/catkin_ws && catkin_make
```

* Konfiguracija mreže
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
## 3. RaspberryPI setup

* Preuzimanje i instalacija Ubuntu MATE 16.04 operacijskog sustava, verzija za Raspberry Pi 3 ->  [Ubuntu_MATE_16.04](https://ubuntu-mate.org/download/)
* Instalacija ROS-a na RaspberryPi uređaju
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/robotis_tools/master/install_ros_kinetic_rp3.sh && chmod 755 ./install_ros_kinetic_rp3.sh && bash ./install_ros_kinetic_rp3.sh
```
nakon instalacije potrebno je rebootati uređaj

* Instalacija potrebnih paketa
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
```
brisanje dijelova koji nam nisu potrebni
```
$ cd ~/catkin_ws/src/turtlebot3
$ sudo rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/
```
instalacija paketa i build
```
$ sudo apt-get install ros-kinetic-rosserial-python ros-kinetic-tf
$ cd ~/catkin_ws && catkin_make
```

* USB i mrežne postavke
```
$ rosrun turtlebot3_bringup create_udev_rules
$ nano ~/.bashrc
```
dodavanje na kraju datoteke (eg. HOST_IP = 192.168.0.101, TARGET_IP=102.168.0.201)
```
export ROS_MASTER_URI=http://HOST_IP:11311
export ROS_HOSTNAME=TARGET_IP
```
```
$ source ~/.bashrc
```
* Omogućavanje SSH
```
Preferences->Raspberry Pi Configuration->Interfaces->SSH Enabled
```

## 4. OpenCR setup

* postavljanje na RaspberryPi uređaju
```
$ export OPENCR_PORT=/dev/ttyACM0
$ export OPENCR_MODEL=segway
$ rm -rf ./opencr_update.tar.bz2
$ wget https://github.com/ROBOTIS-GIT/OpenCR/raw/develop/arduino/opencr_release/shell_update/opencr_update.tar.bz2 && tar -xvf opencr_update.tar.bz2 && cd ./opencr_update && ./update.sh $OPENCR_PORT $OPENCR_MODEL.opencr && cd ..
```
* instalacija Arduino IDE na host računalu
usb i compile postavke
```
$ wget https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/99-opencr-cdc.rules
$ sudo cp ./99-opencr-cdc.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger
$ sudo apt-get install libncurses5-dev:i386
```
preuzimanje i instalacija zadnje verzije [Arduino IDE](https://www.arduino.cc/en/Main/Software)
```
$ cd ~/tools/arduino-1.16.0
$ ./install.sh

$ gedit ~/.bashrc
$ export PATH=$PATH:$HOME/tools/arduino-1.16.0
$ source ~/.bashrc
```
pokretanje Arduina
```
$ arduino
```
* postavke za Arduino
```
File->Preferences->Additional Boards Manager URLs: https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
```
```
 Tools → Board → Boards Manager: Install OpenCR by ROBOTIS
```
```
 Tools → Board → OpenCR Board
```
```
 Tools → Port → /dev/ttyACM0
```

## Programiranje PID controlera na OpenCR pločicu

* preuzimanje Filters knjižnice i dodavanje u arduino IDE -> [Link](https://github.com/JonHub/Filters)
* pokretanje arduino projekta
```
 Files->Open->Segway_project
```
* dodatno podešavanje PID parametara i početnog kuta otklona
* testiranje održavanja ravnoteže

## Pokretanje ROS-a

* postavljanje na RaspberryPi uređaju
```
 $ gedit ~/.bashrc
```
dodavanje na kraj datoteke: 
```
 EXPORT TURTLEBOT3_MODEL=segway
```
```
 $ source ~/.bashrc
```
* pokretanje na udaljenom računalu
```
 $ roscore
```
spajanje na turtlebot

```
 $ ssh turtlebot3@ip
 $ roslaunch turtlebot3_bringup turtlebot3_robot.launch
```
nakon uspješnog podizanja i spajanja ROS-a možemo koristiti funkcionalnosti ROS-a poput vizualizacije podataka
u RVIZ-u ili objavljivanja i praćenja podataka s određenih tema




