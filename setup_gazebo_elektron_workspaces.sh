#!/usr/bin/env bash

#compile gazebo and elektron workspaces in a fakeroot environment
branch_name="melodic-setup-working"
if [ `git branch --list $branch_name` ]
then
    git clone --single-branch -b $branch_name https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall
fi
else
    git clone https://github.com/RCPRG-ros-pkg/RCPRG_rosinstall
fi
cd RCPRG_rosinstall
chmod +x setup.sh
./setup.sh -g -e -F -d build -b Release -i /opt

# coppy compiled workspaces to your /opt directory
sudo cp -r build/opt/ws_gazebo /opt/
sudo cp -r build/opt/ws_elektron /opt/
