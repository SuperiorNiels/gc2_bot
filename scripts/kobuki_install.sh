#!/usr/bin/env bash

KOBUKI_WS=~/kobuki_ws
if [ ! -d "$KOBUKI_WS/src" ]; then
  echo "Installing Kobuki ROS packages..."
  mkdir -p $KOBUKI_WS/src
  cd $KOBUKI_WS/src
  git clone https://github.com/yujinrobot/kobuki
  cd kobuki
  git checkout melodic
  sudo rm -r kobuki_capabilities kobuki
  cd $KOBUKI_WS
  rosdep install --from-paths src --ignore-src -r -y
  catkin_make
  echo "source $KOBUKI_WS/devel/setup.bash" >> ~/.bashrc
else
  echo "Kobuki ROS packages already installed!"
fi
source $KOBUKI_WS/devel/setup.bash
