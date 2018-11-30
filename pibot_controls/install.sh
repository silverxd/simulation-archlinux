#!/bin/bash -e

if [ ${#ROS_DISTRO} -ge 1 ]; then
  echo "Installing ITI0201 simulation..."
  cd $HOME
  rm -rf catkin_ws
  mkdir catkin_ws
  cd catkin_ws
  echo "Cloning simulation repository..."
  git clone https://github.com/iti0201/simulation src
  catkin_make -DCMAKE_CXX_FLAGS="-std=c++11"
  sudo rm -rf /usr/bin/robot_test /usr/bin/script_launch
  DIR=$HOME/catkin_ws/src/pibot_controls
  echo "Creating symlinks..."
  sudo rm -rf /usr/bin/robot_test /usr/bin/script_launch
  sudo ln -s $DIR/robot_test /usr/bin
  sudo ln -s $DIR/script_launch /usr/bin
  echo "Installing python3 packages..."
  sudo pip3 install rospkg catkin_pkg
  echo "Finding python3 directory..."
  PYTHONDIR=~/.local/lib/$(ls ~/.local/lib/ | grep python3)/site-packages
  echo "Python directory is $PYTHONDIR"
  rm -f $PYTHONDIR/PiBot.py
  ln -s ~/catkin_ws/src/pibot_controls/PiBot.py $PYTHONDIR
else
  echo "This script requires ROS to be installed beforehand!"
fi
