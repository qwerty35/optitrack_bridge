#pragma once

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <signal.h>

#ifdef _WIN32
#   include <conio.h>
#else
#   include <unistd.h>
#   include <termios.h>
#endif

#include <vector>

#include <NatNetTypes.h>
#include <NatNetCAPI.h>
#include <NatNetClient.h>
#include <LinearKalmanFilter.h>
#include <tf/transform_broadcaster.h>

enum MessageType{
    POSE,
    ODOMETRY,
    TF
};

class NatNetWrapper {
public:
    NatNetWrapper();
    int run();

    void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
    void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
    void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages

private:
    ros::NodeHandle nh;
    tf::TransformBroadcaster br;
    std::vector<ros::Publisher> pubs_vision_pose;
    std::vector<ros::Publisher> pubs_vision_odom;
    std::vector<ros::Publisher> pubs_labeled_marker_pose_array;
    ros::Publisher pub_unlabeled_marker_pose_array;
    std::vector<std::unique_ptr<LinearKalmanFilter>> linearKalmanFilters;
    std::vector<int> model_ids;
    std::vector<std::string> model_names;
    std::string prefix;
    std::string frame_id;
    MessageType message_type;
    int verbose_level;
    bool show_latency = false;
    bool publish_labeled_marker_pose_array = false;
    bool publish_unlabeled_marker_pose_array = false;
    bool is_ServerDiscovered = false;

    static void sigintCallback(int signum);
    void resetClient();
    int ConnectClient();
    geometry_msgs::PoseStamped rigidBodyToPose(const sRigidBodyData& rigid_body_data);
    void publishPose(int idx, const sRigidBodyData& rigid_body_data);
    void publishOdom(int idx, const sRigidBodyData& rigid_body_data);
    void publishTF(int idx, const sRigidBodyData& rigid_body_data);

    static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

    char getch();
    NatNetClient* g_pClient = NULL;

    std::vector< sNatNetDiscoveredServer > g_discoveredServers;
    sNatNetClientConnectParams g_connectParams;
    char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
    int g_analogSamplesPerMocapFrame = 0;
    sServerDescription g_serverDescription;
};

