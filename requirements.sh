#!/bin/bash

# exit when any command fails
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG

# echo an error message before exiting
trap 'echo "\"${last_command}\" command filed with exit code $?."' EXIT

# apt-get installs
sudo apt update
sudo apt install git -y
sudo apt install python-rospy -y
sudo apt install python-pip -y
sudo apt install ros-kinetic-rosserial -y
sudo apt install ros-kinetic-rosserial-arduino -y

# pip installs
pip install --upgrade pip
pip install numpy
