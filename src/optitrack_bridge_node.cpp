#include "NatNetWrapper.h"

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "optitrack_bridge_node");
    NatNetWrapper client;
    client.run();
}