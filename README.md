# optitrack_bridge

optitrack bridge for ROS Kinetic, Ubuntu 16.04

1.Note
------
This code is for Motive 2.2, NatNet 3.1.

This code is based on the NatNet 3.1 SDK (https://optitrack.com/products/natnet-sdk/)


2.Installation
------
    cd ~/catkin/src (or your workspace)

    git clone https://github.com/qwerty35/optitrack_bridge.git

catkin build


3.Usage
------
    roslaunch optitrack_bridge optitrack.launch

then check rostopic list
