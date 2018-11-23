#!/usr/bin/env bash

#compile gazebo and elektron workspaces in a fakeroot environment
git clone https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall
cd RCPRG_rosinstall
chmod +x setup.sh
./setup.sh -g -e -F -d build -b Release -i /opt

# coppy compiled workspaces to your /opt directory
sudo cp -r build/opt/ws_gazebo /opt/
sudo cp -r build/opt/ws_elektron /opt/
