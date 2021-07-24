# optitrack_bridge

This package converts Optitrack rigidBody to ROS message.

Tested in 
- Ubuntu 20.04, ROS Noetic 
- Ubuntu 18.04, ROS Melodic 
- Ubuntu 16.04, ROS Kinetic


1.Prerequisite
------
This code is for below environment.
- C++14
- Motive 2.2 or 2.3
- NatNet 3.1

2.Installation
------
    cd ~/catkin_ws/src

    git clone https://github.com/qwerty35/optitrack_bridge.git

    cd .. && catkin_make

It assumes that your work space is in `~/catkin_ws/src`

3.Usage
------
    roslaunch optitrack_bridge optitrack.launch


4.Parameters
-----
`frame_id`: Set frame id of message.

`message_type`: Ros message type.

+ pose - It returns object's pose as geometry::poseStamped message.
+ odometry - It returns object's pose+twist as nav_msgs::odometry message. The twist of object is computed by linear Kalman filter. (Error covariance is not supported yet.)
+ tf - It returns object's pose as tf.

`show_latency`: Print latency on the screen.

`publish_labeled_marker_pose_array`: If true, publish pose of object's markers as geometry::poseArray 

`publish_unlabeled_marker_pose_array`: If true, publish unlabeled markers as geometry::poseArray

(If labeled or unlabeled markers are not published, check Motive -> Streaming Pane -> Labeled or Unlabeld Markers)
