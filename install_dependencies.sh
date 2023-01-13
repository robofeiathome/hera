#!/bin/bash

distribution=${1:-noetic} 

apt-get install -y ros-$distribution-teleop-twist-keyboard
apt-get install -y ros-$distribution-gmapping
apt-get install -y ros-$distribution-map-server
apt-get install -y ros-$distribution-move-base
apt-get install -y ros-$distribution-social-navigation-layers
apt-get install -y ros-$distribution-amcl
apt-get install -y ros-$distribution-fake-localization
apt-get install -y ros-$distribution-carrot-planner
apt-get install -y ros-$distribution-global-planner
apt-get install -y ros-$distribution-dwa-local-planner
apt-get install -y ros-$distribution-eband-local-planner
apt-get install -y ros-$distribution-teb-local-planner

# sudo apt-get install python-enum34

pip install face_recognition
pip install translate
