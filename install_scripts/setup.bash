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

# Check for python-wstool package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' python-wstool|grep "install ok installed")
echo Checking for python-wstool: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No python-wstool. Setting up python-wstool."
  sudo apt-get --yes install python-wstool
fi

# Check for ros-melodic-robot-localization package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-robot-localization|grep "install ok installed")
echo Checking for ros-melodic-robot-localization: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-robot-localization. Setting up ros-melodic-robot-localization."
  sudo apt-get --yes install ros-melodic-robot-localization 
fi

# Check for ros-melodic-teb-local-planner package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-teb-local-planner|grep "install ok installed")
echo Checking for ros-melodic-teb-local-planner: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-teb-local-planner. Setting up ros-melodic-teb-local-planner."
  sudo apt-get --yes install ros-melodic-teb-local-planner
fi

# Check for ros-melodic-eband-local-planner package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-eband-local-planner|grep "install ok installed")
echo Checking for ros-melodic-eband-local-planner: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-eband-local-planner. Setting up ros-melodic-eband-local-planner."
  sudo apt-get --yes install ros-melodic-eband-local-planner
fi

# Check for ros-melodic-hector-slam package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-hector-slam|grep "install ok installed")
echo Checking for ros-melodic-hector-slam: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-hector-slam. Setting up ros-melodic-hector-slam."
  sudo apt-get --yes install ros-melodic-hector-slam
fi

# Check for ros-melodic-rqt-multiplot package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-rqt-multiplot|grep "install ok installed")
echo Checking for ros-melodic-rqt-multiplot: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-rqt-multiplot. Setting up ros-melodic-rqt-multiplot."
  sudo apt-get --yes install ros-melodic-rqt-multiplot
fi

# Camera packages not used currently
# Check for ros-melodic-openni-camera package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-openni-camera|grep "install ok installed")
echo Checking for ros-melodic-openni-camera: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-openni-camera. Setting up ros-melodic-openni-camera."
  sudo apt-get --yes install ros-melodic-openni-camera
fi

# Check for ros-melodic-openni-launch package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-openni-launch|grep "install ok installed")
echo Checking for ros-melodic-openni-launch: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-openni-launch. Setting up ros-melodic-openni-launch."
  sudo apt-get --yes install ros-melodic-openni-launch
fi

# Check for ros-melodic-rtabmap-ros package
PKG_OK=$(dpkg-query -W --showformat='${Status}\n' ros-melodic-rtabmap-ros|grep "install ok installed")
echo Checking for ros-melodic-rtabmap-ros: $PKG_OK
if [ -z "$PKG_OK" ]; then
  echo "No ros-melodic-rtabmap-ros. Setting up ros-melodic-rtabmap-ros."
  sudo apt-get --yes install ros-melodic-rtabmap-ros
fi

cd $1
mkdir src
wstool init
wstool merge $2
wstool update

catkin build