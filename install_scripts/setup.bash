#!/bin/sh

if [ "$#" -ne 2 ]; then
  echo "Usage: $0 [root of new workspace] [FULL path to the velmwheel rosinstall file]" >&2
  exit 1
fi
if ! [ -e "$1" ]; then
  echo "$1 not found" >&2
  exit 1
fi
if ! [ -d "$1" ]; then
  echo "$1 not a directory" >&2
  exit 1
fi

if ! [ -e "$2" ]; then
  echo "$2 not found" >&2
  exit 1
fi
if ! [ -f "$2" ]; then
  echo "$2 not a directory" >&2
  exit 1
fi
# update packages sources
sudo apt-get update

# Check for python-wstool package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' python3-wstool|grep "install ok installed")
echo Checking for python3-wstool: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No python-wstool. Setting up python-wstool."
  sudo apt-get --yes install python3-wstool
fi

# Check for ros-noetic-robot-localization package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-robot-localization|grep "install ok installed")
echo Checking for ros-noetic-robot-localization: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-robot-localization. Setting up ros-noetic-robot-localization."
  sudo apt-get --yes install ros-noetic-robot-localization 
fi

# Check for ros-noetic-teb-local-planner package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-teb-local-planner|grep "install ok installed")
echo Checking for ros-noetic-teb-local-planner: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-teb-local-planner. Setting up ros-noetic-teb-local-planner."
  sudo apt-get --yes install ros-noetic-teb-local-planner
fi

# Check for python3-catkin-tools package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' python3-catkin-tools|grep "install ok installed")
echo Checking for python3-catkin-tools: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No rpython3-catkin-tools. Setting up rpython3-catkin-tools."
  sudo apt-get --yes install python3-catkin-tools
fi

# Check for ros-noetic-move-base package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-move-base|grep "install ok installed")
echo Checking for ros-noetic-move-base: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-move-base. Setting up ros-noetic-move-base."
  sudo apt-get --yes install ros-noetic-move-base
fi

# Check for ros-noetic-slam-toolbox package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-move-base|grep "install ok installed")
echo Checking for ros-noetic-slam-toolbox: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-slam-toolbox. Setting up ros-noetic-slam-toolbox."
  sudo apt-get --yes install ros-noetic-slam-toolbox
fi

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-map-server|grep "install ok installed")
echo Checking for ros-noetic-map-server: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-map-server. Setting up ros-noetic-map-server."
  sudo apt-get --yes install ros-noetic-map-server
fi

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-teleop-twist-keyboard|grep "install ok installed")
echo Checking for ros-noetic-teleop-twist-keyboard: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-teleop-twist-keyboard. Setting up ros-noetic-teleop-twist-keyboard."
  sudo apt-get --yes install ros-noetic-teleop-twist-keyboard
fi

# Check for ros-noetic-amcl package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-amcl|grep "install ok installed")
echo Checking for ros-noetic-amcl: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-noetic-amcl. Setting up ros-noetic-amcl."
  sudo apt-get --yes install ros-noetic-amcl
fi

# Check for ros-noetic-eband-local-planner package
# PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-eband-local-planner|grep "install ok installed")
# echo Checking for ros-noetic-eband-local-planner: $PKG_OK
# if [ -z "$PKG_OK" ]; then
#   echo "No ros-noetic-eband-local-planner. Setting up ros-noetic-eband-local-planner."
#   sudo apt-get --yes install ros-noetic-eband-local-planner
# fi

# # Check for ros-noetic-hector-slam package
# PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-hector-slam|grep "install ok installed")
# echo Checking for ros-noetic-hector-slam: $PKG_OK
# if [ -z "$PKG_OK" ]; then
#   echo "No ros-noetic-hector-slam. Setting up ros-noetic-hector-slam."
#   sudo apt-get --yes install ros-noetic-hector-slam
# fi

# Check for ros-noetic-rqt-multiplot package
# PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-rqt-multiplot|grep "install ok installed")
# echo Checking for ros-noetic-rqt-multiplot: $PKG_OK
# if [ -z "$PKG_OK" ]; then
#   echo "No ros-noetic-rqt-multiplot. Setting up ros-noetic-rqt-multiplot."
#   sudo apt-get --yes install ros-noetic-rqt-multiplot
# fi

# Camera packages not used currently
# Check for ros-noetic-openni-camera package
# PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-openni-camera|grep "install ok installed")
# echo Checking for ros-noetic-openni-camera: $PKG_OK
# if [ -z "$PKG_OK" ]; then
#   echo "No ros-noetic-openni-camera. Setting up ros-noetic-openni-camera."
#   sudo apt-get --yes install ros-noetic-openni-camera
# fi

# Check for ros-noetic-openni-launch package
# PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-openni-launch|grep "install ok installed")
# echo Checking for ros-noetic-openni-launch: $PKG_OK
# if [ -z "$PKG_OK" ]; then
#   echo "No ros-noetic-openni-launch. Setting up ros-noetic-openni-launch."
#   sudo apt-get --yes install ros-noetic-openni-launch
# fi

# # Check for ros-noetic-rtabmap-ros package
# PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-noetic-rtabmap-ros|grep "install ok installed")
# echo Checking for ros-noetic-rtabmap-ros: $PKG_OK
# if [ -z "$PKG_OK" ]; then
#   echo "No ros-noetic-rtabmap-ros. Setting up ros-noetic-rtabmap-ros."
#   sudo apt-get --yes install ros-noetic-rtabmap-ros
# fi

cd $1
mkdir src
wstool init
wstool merge $2
wstool update

catkin build
