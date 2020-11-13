# optitrack_bridge

This package converts Optitrack object pose to ROS message.

Tested in [ROS Melodic, Ubuntu 18.04], [ROS Kinetic, Ubuntu 16.04]


1.Note
------
This code is for C++14, Motive 2.2, NatNet 3.1.

This code is based on the NatNet 3.1 SDK (https://optitrack.com/products/natnet-sdk/)



2.Installation
------
    cd ~/catkin/src (or your workspace)

    git clone https://github.com/qwerty35/optitrack_bridge.git

    cd .. && catkin_make (or catkin build optitrack_bridge)



3.Usage
------
    roslaunch optitrack_bridge optitrack.launch


4.Parameters
-----
"frame_id": set frame id of message.

"show_latency": print latency on the screen.

"publish_with_twist":

+ True - it returns object's pose+twist as nav_msgs::odometry message. The twist of object is computed by linear Kalman filter

+ False - it returns object's pose as geometry::poseStamped message.

"publish_labeled_marker_pose_array": if true, publish pose of object's markers as geometry::poseArray 

"publish_unlabeled_marker_pose_array": if true, publish unlabeled markers as geometry::poseArray

(If labeled or unlabeled markers are not published, check Motive -> Streaming Pane -> Labeled or Unlabeld Markers)
