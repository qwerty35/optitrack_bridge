#pragma once

#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

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

class NatNetWrapper {
public:
    NatNetWrapper();
    int run();

    void NATNET_CALLCONV ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext );
    void NATNET_CALLCONV DataHandler(sFrameOfMocapData* data, void* pUserData);    // receives data from the server
    void NATNET_CALLCONV MessageHandler(Verbosity msgType, const char* msg);      // receives NatNet error messages

private:
    ros::NodeHandle nh;
    std::vector<ros::Publisher> pubs_vision_pose;
    std::string prefix;
    std::string frame_id;
    bool showLatency;
    int verbose_level;

    bool is_ServerDiscovered;

    void resetClient();
    int ConnectClient();

    static const ConnectionType kDefaultConnectionType = ConnectionType_Multicast;

    char getch();
    NatNetClient* g_pClient = NULL;

    std::vector< sNatNetDiscoveredServer > g_discoveredServers;
    sNatNetClientConnectParams g_connectParams;
    char g_discoveredMulticastGroupAddr[kNatNetIpv4AddrStrLenMax] = NATNET_DEFAULT_MULTICAST_ADDRESS;
    int g_analogSamplesPerMocapFrame = 0;
    sServerDescription g_serverDescription;
};

