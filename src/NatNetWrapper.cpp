#include "NatNetWrapper.h"

std::atomic<bool> quit(false); // signal flag

NatNetWrapper* NatNetWrapper_ptr;
void logCallbackWrapper( Verbosity level, const char* message ){
    return NatNetWrapper_ptr->MessageHandler(level, message);
}
void dataCallbackWrapper( sFrameOfMocapData* pFrameOfData, void* pUserData ){
    return NatNetWrapper_ptr->DataHandler(pFrameOfData, pUserData);
}
void serverCallbackWrapper( const sNatNetDiscoveredServer* pNewServer, void* pUserContext ){
    return NatNetWrapper_ptr->ServerDiscoveredCallback(pNewServer, pUserContext);
}

NatNetWrapper::NatNetWrapper(){
    NatNetWrapper_ptr = this;
    std::string message_type_;
    nh = ros::NodeHandle("~");
    nh.param<std::string>("frame_id", frame_id, "/world");
    nh.param<std::string>("message_type", message_type_, "pose");
    nh.param<bool>("show_latency", show_latency, false);
    nh.param<bool>("publish_labeled_marker_pose_array", publish_labeled_marker_pose_array, false);
    nh.param<bool>("publish_unlabeled_marker_pose_array", publish_unlabeled_marker_pose_array, false);
    prefix = "/optitrack/";
    verbose_level = Verbosity_Error + 1; // Do not listen NatNetlib message

    if(message_type_ == "pose"){
        message_type = MessageType::POSE;
    }
    else if(message_type_ == "odom"){
        message_type = MessageType::ODOMETRY;
    }
    else if(message_type_ == "tf"){
        message_type = MessageType::TF;
    }
    else{
        ROS_ERROR("[NatNetWrapper] Invalid message type. message type must be pose, twist, or tf");
        throw std::invalid_argument("message_type");
    }
}

int NatNetWrapper::run() {
    // print version info
    unsigned char ver[4];
    NatNet_GetVersion( ver );
    printf( "NatNetWrapper (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3] );

    // Install logging callback
    NatNet_SetLogCallback( logCallbackWrapper );

    // create NatNet client
    g_pClient = new NatNetClient();

    // set the frame callback handler
    g_pClient->SetFrameReceivedCallback( dataCallbackWrapper, g_pClient );	// this function will receive data from the server

    // If no arguments were specified on the command line,
    // attempt to discover servers on the local network.

    // Do asynchronous server discovery.
    printf( "Looking for servers on the local network.\n" );
    is_ServerDiscovered = false;
    NatNetDiscoveryHandle discovery;
    NatNet_CreateAsyncServerDiscovery( &discovery, serverCallbackWrapper );

    ros::Rate server_wait_rate(10);
    while(ros::ok() and !is_ServerDiscovered) {
        server_wait_rate.sleep();
    }

    const size_t serverIndex = 0;

    const sNatNetDiscoveredServer& discoveredServer = g_discoveredServers[serverIndex];

    if ( discoveredServer.serverDescription.bConnectionInfoValid )
    {
        // Build the connection parameters.
#ifdef _WIN32
        _snprintf_s(
#else
        snprintf(
#endif
                g_discoveredMulticastGroupAddr, sizeof g_discoveredMulticastGroupAddr,
                "%" PRIu8 ".%" PRIu8".%" PRIu8".%" PRIu8"",
                discoveredServer.serverDescription.ConnectionMulticastAddress[0],
                discoveredServer.serverDescription.ConnectionMulticastAddress[1],
                discoveredServer.serverDescription.ConnectionMulticastAddress[2],
                discoveredServer.serverDescription.ConnectionMulticastAddress[3]
        );

        g_connectParams.connectionType = discoveredServer.serverDescription.ConnectionMulticast ? ConnectionType_Multicast : ConnectionType_Unicast;
        g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
        g_connectParams.serverDataPort = discoveredServer.serverDescription.ConnectionDataPort;
        g_connectParams.serverAddress = discoveredServer.serverAddress;
        g_connectParams.localAddress = discoveredServer.localAddress;
        g_connectParams.multicastAddress = g_discoveredMulticastGroupAddr;
    }
    else
    {
        // We're missing some info because it's a legacy server.
        // Guess the defaults and make a best effort attempt to connect.
        g_connectParams.connectionType = kDefaultConnectionType;
        g_connectParams.serverCommandPort = discoveredServer.serverCommandPort;
        g_connectParams.serverDataPort = 0;
        g_connectParams.serverAddress = discoveredServer.serverAddress;
        g_connectParams.localAddress = discoveredServer.localAddress;
        g_connectParams.multicastAddress = NULL;
    }

    NatNet_FreeAsyncServerDiscovery( discovery );

    int iResult;

    // Connect to Motive
    iResult = ConnectClient();
    if (iResult != ErrorCode_OK)
    {
        printf("Error initializing client.  See log for details.  Exiting");
        return 1;
    }
    else
    {
        printf("Client initialized and ready.\n");
    }


    // Send/receive test request
    void* response;
    int nBytes;
//    printf("[NatNetWrapper] Sending Test Request\n");
    iResult = g_pClient->SendMessageAndWait("TestRequest", &response, &nBytes);
    if (iResult == ErrorCode_OK)
    {
        printf("[NatNetWrapper] Received: %s", (char*)response);
    }

    // Retrieve Data Descriptions from Motive
    printf("\n[NatNetWrapper] Requesting Data Descriptions...\n");
    sDataDescriptions* pDataDefs = NULL;
    iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
    if (iResult != ErrorCode_OK || pDataDefs == NULL)
    {
        printf("[NatNetWrapper] Unable to retrieve Data Descriptions.");
    }
    else
    {
//        printf("[NatNetWrapper] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions );
        for(int i=0; i < pDataDefs->nDataDescriptions; i++)
        {
//            printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
            {
                // MarkerSet
                sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
                printf("MarkerSet Name : %s\n", pMS->szName);
                for(int j = 0; j < pMS->nMarkers; j++)
                    printf("%s\n", pMS->szMarkerNames[j]);

            }
            if(pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
            {
                // RigidBody
                sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
                printf("RigidBody Name : %s\n", pRB->szName);
                printf("RigidBody ID : %d\n", pRB->ID);
//                printf("RigidBody Parent ID : %d\n", pRB->parentID);
//                printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);

                model_ids.emplace_back(pRB->ID);
                model_names.emplace_back(std::string(pRB->szName));

                // Initialize publisher by message type
                if(message_type == MessageType::POSE){
                    pubs_vision_pose.emplace_back(nh.advertise<geometry_msgs::PoseStamped>(prefix + std::string(pRB->szName) + "/poseStamped", 10));
                }
                else if(message_type == MessageType::ODOMETRY){
                    pubs_vision_odom.emplace_back(nh.advertise<nav_msgs::Odometry>(prefix + std::string(pRB->szName) + "/odometry", 10));
                    linearKalmanFilters.emplace_back(std::make_unique<LinearKalmanFilter>());
                }

                if(publish_labeled_marker_pose_array){
                    pubs_labeled_marker_pose_array.emplace_back(nh.advertise<geometry_msgs::PoseArray>(prefix + std::string(pRB->szName) + "/markerPoseArray", 10));
                }

//                if ( pRB->MarkerPositions != NULL && pRB->MarkerRequiredLabels != NULL )
//                {
//                    for ( int markerIdx = 0; markerIdx < pRB->nMarkers; ++markerIdx )
//                    {
//                        const MarkerData& markerPosition = pRB->MarkerPositions[markerIdx];
//                        const int markerRequiredLabel = pRB->MarkerRequiredLabels[markerIdx];
//
//                        printf( "\tMarker #%d:\n", markerIdx );
//                        printf( "\t\tPosition: %.2f, %.2f, %.2f\n", markerPosition[0], markerPosition[1], markerPosition[2] );
//
//                        if ( markerRequiredLabel != 0 )
//                        {
//                            printf( "\t\tRequired active label: %d\n", markerRequiredLabel );
//                        }
//                    }
//                }
            }
//            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
//            {
//                // Skeleton
//                sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
//                printf("Skeleton Name : %s\n", pSK->szName);
//                printf("Skeleton ID : %d\n", pSK->skeletonID);
//                printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
//                for(int j=0; j < pSK->nRigidBodies; j++)
//                {
//                    sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
//                    printf("  RigidBody Name : %s\n", pRB->szName);
//                    printf("  RigidBody ID : %d\n", pRB->ID);
//                    printf("  RigidBody Parent ID : %d\n", pRB->parentID);
//                    printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
//                }
//            }
//            else if(pDataDefs->arrDataDescriptions[i].type == Descriptor_ForcePlate)
//            {
//                // Force Plate
//                sForcePlateDescription* pFP = pDataDefs->arrDataDescriptions[i].Data.ForcePlateDescription;
//                printf("Force Plate ID : %d\n", pFP->ID);
//                printf("Force Plate Serial : %s\n", pFP->strSerialNo);
//                printf("Force Plate Width : %3.2f\n", pFP->fWidth);
//                printf("Force Plate Length : %3.2f\n", pFP->fLength);
//                printf("Force Plate Electrical Center Offset (%3.3f, %3.3f, %3.3f)\n", pFP->fOriginX,pFP->fOriginY, pFP->fOriginZ);
//                for(int iCorner=0; iCorner<4; iCorner++)
//                    printf("Force Plate Corner %d : (%3.4f, %3.4f, %3.4f)\n", iCorner, pFP->fCorners[iCorner][0],pFP->fCorners[iCorner][1],pFP->fCorners[iCorner][2]);
//                printf("Force Plate Type : %d\n", pFP->iPlateType);
//                printf("Force Plate Data Type : %d\n", pFP->iChannelDataType);
//                printf("Force Plate Channel Count : %d\n", pFP->nChannels);
//                for(int iChannel=0; iChannel<pFP->nChannels; iChannel++)
//                    printf("\tChannel %d : %s\n", iChannel, pFP->szChannelNames[iChannel]);
//            }
//            else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Device)
//            {
//                // Peripheral Device
//                sDeviceDescription* pDevice = pDataDefs->arrDataDescriptions[i].Data.DeviceDescription;
//                printf("Device Name : %s\n", pDevice->strName);
//                printf("Device Serial : %s\n", pDevice->strSerialNo);
//                printf("Device ID : %d\n", pDevice->ID);
//                printf("Device Channel Count : %d\n", pDevice->nChannels);
//                for (int iChannel = 0; iChannel < pDevice->nChannels; iChannel++)
//                    printf("\tChannel %d : %s\n", iChannel, pDevice->szChannelNames[iChannel]);
//            }
//            else
//            {
//                printf("Unknown data type.");
//                // Unknown
//            }
        }
        if(publish_unlabeled_marker_pose_array){
            pub_unlabeled_marker_pose_array = nh.advertise<geometry_msgs::PoseArray>(prefix + "unlabeled/markerPoseArray", 10);
        }
    }

    // Ready to receive marker stream!
    signal(SIGINT, NatNetWrapper::sigintCallback);
    printf("\nClient is connected to server and listening for data...\n");
    int c;
    bool bExit = false;
    ros::Rate main_rate(100);
    while(ros::ok())
    {
//        c=getch();
//        switch(c)
//        {
//            case 'q':
//                bExit = true;
//                break;
//            case 'r':
//                resetClient();
//                break;
//            case 'p':
//                sServerDescription ServerDescription;
//                memset(&ServerDescription, 0, sizeof(ServerDescription));
//                g_pClient->GetServerDescription(&ServerDescription);
//                if(!ServerDescription.HostPresent)
//                {
//                    printf("Unable to connect to server. Host not present. Exiting.");
//                    return 1;
//                }
//                break;
//            case 's':
//            {
//                printf("\n\n[NatNetWrapper] Requesting Data Descriptions...");
//                sDataDescriptions* pDataDefs = NULL;
//                iResult = g_pClient->GetDataDescriptionList(&pDataDefs);
//                if (iResult != ErrorCode_OK || pDataDefs == NULL)
//                {
//                    printf("[NatNetWrapper] Unable to retrieve Data Descriptions.");
//                }
//                else
//                {
//                    printf("[NatNetWrapper] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
//                }
//            }
//                break;
//            case 'm':	                        // change to multicast
//                g_connectParams.connectionType = ConnectionType_Multicast;
//                iResult = ConnectClient();
//                if(iResult == ErrorCode_OK)
//                    printf("Client connection type changed to Multicast.\n\n");
//                else
//                    printf("Error changing client connection type to Multicast.\n\n");
//                break;
//            case 'u':	                        // change to unicast
//                g_connectParams.connectionType = ConnectionType_Unicast;
//                iResult = ConnectClient();
//                if(iResult == ErrorCode_OK)
//                    printf("Client connection type changed to Unicast.\n\n");
//                else
//                    printf("Error changing client connection type to Unicast.\n\n");
//                break;
//            case 'c' :                          // connect
//                iResult = ConnectClient();
//                break;
//            case 'd' :                          // disconnect
//                // note: applies to unicast connections only - indicates to Motive to stop sending packets to that client endpoint
//                iResult = g_pClient->SendMessageAndWait("Disconnect", &response, &nBytes);
//                if (iResult == ErrorCode_OK)
//                    printf("[NatNetWrapper] Disconnected");
//                break;
//            default:
//                break;
//        }
//        if(bExit)
//            break;

        if(quit.load())
            break;

        main_rate.sleep();
    }

    // Done - clean up.
    if (g_pClient)
    {
        g_pClient->Disconnect();
        delete g_pClient;
        g_pClient = NULL;
    }

    return ErrorCode_OK;
}

// DataHandler receives data from the server
// This function is called by NatNet when a frame of mocap data is available
void NATNET_CALLCONV NatNetWrapper::DataHandler(sFrameOfMocapData* data, void* pUserData)
{
    NatNetClient* pClient = (NatNetClient*) pUserData;

    // Software latency here is defined as the span of time between:
    //   a) The reception of a complete group of 2D frames from the camera system (CameraDataReceivedTimestamp)
    // and
    //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
    //
    // This figure may appear slightly higher than the "software latency" reported in the Motive user interface,
    // because it additionally includes the time spent preparing to stream the data via NatNet.
    const uint64_t softwareLatencyHostTicks = data->TransmitTimestamp - data->CameraDataReceivedTimestamp;
    const double softwareLatencyMillisec = (softwareLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

    // Transit latency is defined as the span of time between Motive transmitting the frame of data, and its reception by the client (now).
    // The SecondsSinceHostTimestamp method relies on NatNetClient's internal clock synchronization with the server using Cristian's algorithm.
    const double transitLatencyMillisec = pClient->SecondsSinceHostTimestamp( data->TransmitTimestamp ) * 1000.0;

    int i=0;

//    printf("FrameID : %d\n", data->iFrame);
//    printf("Timestamp : %3.2lf\n", data->fTimestamp);


    // Only recent versions of the Motive software in combination with ethernet camera systems support system latency measurement.
    // If it's unavailable (for example, with USB camera systems, or during playback), this field will be zero.
    const bool bSystemLatencyAvailable = data->CameraMidExposureTimestamp != 0;

    if(show_latency) {
        if (bSystemLatencyAvailable) {
            // System latency here is defined as the span of time between:
            //   a) The midpoint of the camera exposure window, and therefore the average age of the photons (CameraMidExposureTimestamp)
            // and
            //   b) The time immediately prior to the NatNet frame being transmitted over the network (TransmitTimestamp)
            const uint64_t systemLatencyHostTicks = data->TransmitTimestamp - data->CameraMidExposureTimestamp;
            const double systemLatencyMillisec =
                    (systemLatencyHostTicks * 1000) / static_cast<double>(g_serverDescription.HighResClockFrequency);

            // Client latency is defined as the sum of system latency and the transit time taken to relay the data to the NatNet client.
            // This is the all-inclusive measurement (photons to client processing).
            const double clientLatencyMillisec =
                    pClient->SecondsSinceHostTimestamp(data->CameraMidExposureTimestamp) * 1000.0;

            // You could equivalently do the following (not accounting for time elapsed since we calculated transit latency above):
            //const double clientLatencyMillisec = systemLatencyMillisec + transitLatencyMillisec;

            printf("Software latency : %.2lf milliseconds\n", softwareLatencyMillisec);
            printf("System latency : %.2lf milliseconds\n", systemLatencyMillisec);
            printf("Total client latency : %.2lf milliseconds (transit time +%.2lf ms)\n", clientLatencyMillisec,
                   transitLatencyMillisec);
        } else {
            printf("Transit latency : %.2lf milliseconds\n", transitLatencyMillisec);
        }
    }

    // FrameOfMocapData params
    bool bIsRecording = ((data->params & 0x01)!=0);
    bool bTrackedModelsChanged = ((data->params & 0x02)!=0);
    if(bIsRecording)
        printf("RECORDING\n");
    if(bTrackedModelsChanged)
        printf("Models Changed.\n");


    // timecode - for systems with an eSync and SMPTE timecode generator - decode to values
    int hour, minute, second, frame, subframe;
    NatNet_DecodeTimecode( data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe );
    // decode to friendly string
    char szTimecode[128] = "";
    NatNet_TimecodeStringify( data->Timecode, data->TimecodeSubframe, szTimecode, 128 );
//    printf("Timecode : %s\n", szTimecode);

    // Rigid Bodies
//    printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);


    for(i = 0; i < data->nRigidBodies; i++) {
        if(model_ids.size() != data->nRigidBodies){ // Do not publish when number of data is not matched
            break;
        }

        // params
        // 0x01 : bool, rigid body was successfully tracked in this frame
        bool bTrackingValid = data->RigidBodies[i].params & 0x01;
//        printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError, bTrackingValid);
        if (bTrackingValid) {
            switch (message_type){
                case POSE:
                    publishPose(i, data->RigidBodies[i]);
                    break;
                case ODOMETRY:
                    publishOdom(i, data->RigidBodies[i]);
                    break;
                case TF:
                    publishTF(i, data->RigidBodies[i]);
                    break;
                default:
                    break;
            }
        } else {
            ROS_WARN_STREAM("[NatNetWrapper] " << model_names[i] << " is not published");
        }
    }

//    // Skeletons
//    printf("Skeletons [Count=%d]\n", data->nSkeletons);
//    for(i=0; i < data->nSkeletons; i++)
//    {
//        sSkeletonData skData = data->Skeletons[i];
//        printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
//        for(int j=0; j< skData.nRigidBodies; j++)
//        {
//            sRigidBodyData rbData = skData.RigidBodyData[j];
//            printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
//                   rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw );
//        }
//    }

    // labeled markers - this includes all markers (Active, Passive, and 'unlabeled' (markers with no asset but a PointCloud ID)
    if(publish_labeled_marker_pose_array || publish_unlabeled_marker_pose_array){
        std::vector<geometry_msgs::PoseArray> labeled_marker_pose_array;
        labeled_marker_pose_array.resize(model_ids.size());
        geometry_msgs::PoseArray unlabeled_marker_pose_array;
        bool bOccluded;     // marker was not visible (occluded) in this frame
        bool bPCSolved;     // reported position provided by point cloud solve
        bool bModelSolved;  // reported position provided by model solve
        bool bHasModel;     // marker has an associated asset in the data stream
        bool bUnlabeled;    // marker is 'unlabeled', but has a point cloud ID that matches Motive PointCloud ID (In Motive 3D View)
        bool bActiveMarker; // marker is an actively labeled LED marker

//    printf("Markers [Count=%d]\n", data->nLabeledMarkers);
        for(i=0; i < data->nLabeledMarkers; i++) {
            bOccluded = ((data->LabeledMarkers[i].params & 0x01) != 0);
            bPCSolved = ((data->LabeledMarkers[i].params & 0x02) != 0);
            bModelSolved = ((data->LabeledMarkers[i].params & 0x04) != 0);
            bHasModel = ((data->LabeledMarkers[i].params & 0x08) != 0);
            bUnlabeled = ((data->LabeledMarkers[i].params & 0x10) != 0);
            bActiveMarker = ((data->LabeledMarkers[i].params & 0x20) != 0);

            sMarker marker = data->LabeledMarkers[i];

            // Marker ID Scheme:
            // Active Markers:
            //   ID = ActiveID, correlates to RB ActiveLabels list
            // Passive Markers:
            //   If Asset with Legacy Labels
            //      AssetID 	(Hi Word)
            //      MemberID	(Lo Word)
            //   Else
            //      PointCloud ID
            int modelID, markerID;
            NatNet_DecodeID(marker.ID, &modelID, &markerID);

//        char szMarkerType[512];
//        if (bActiveMarker)
//            strcpy(szMarkerType, "Active");
//        else if(bUnlabeled)
//            strcpy(szMarkerType, "Unlabeled");
//        else
//            strcpy(szMarkerType, "Labeled");
//
//        printf("%s Marker [ModelID=%d, MarkerID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
//               szMarkerType, modelID, markerID, bOccluded, bPCSolved, bModelSolved,  marker.size, marker.x, marker.y, marker.z);

            geometry_msgs::Pose marker_pose;
            marker_pose.position.x = marker.x;
            marker_pose.position.y = marker.y;
            marker_pose.position.z = marker.z;
            marker_pose.orientation.x = 0;
            marker_pose.orientation.y = 0;
            marker_pose.orientation.z = 0;
            marker_pose.orientation.w = 1;

            if (bActiveMarker) {
                //TODO: ActiveMarker
            } else if (bUnlabeled && publish_unlabeled_marker_pose_array) {
                unlabeled_marker_pose_array.header.stamp = ros::Time::now();
                unlabeled_marker_pose_array.header.frame_id = frame_id;
                unlabeled_marker_pose_array.poses.push_back(marker_pose);
            } else if(publish_labeled_marker_pose_array){
                int labeled_marker_index = -1;
                for(int i_model = 0; i_model < model_ids.size(); i_model++){
                    if(modelID == model_ids[i_model]){
                        labeled_marker_index = i_model;
                        break;
                    }
                }
                if(labeled_marker_index == -1){
                    // data is not ready
                    break;
                }

                labeled_marker_pose_array[labeled_marker_index].header.stamp = ros::Time::now();
                labeled_marker_pose_array[labeled_marker_index].header.frame_id = frame_id;
                labeled_marker_pose_array[labeled_marker_index].poses.emplace_back(marker_pose);
            }
        }
        if(publish_labeled_marker_pose_array && !pubs_labeled_marker_pose_array.empty()){
            for(int i_model=0; i_model < model_ids.size(); i_model++){
                pubs_labeled_marker_pose_array[i_model].publish(labeled_marker_pose_array[i_model]);
            }
        }
        if(publish_unlabeled_marker_pose_array){
            pub_unlabeled_marker_pose_array.publish(unlabeled_marker_pose_array);
        }
    }

//    // force plates
//    printf("Force Plate [Count=%d]\n", data->nForcePlates);
//    for(int iPlate=0; iPlate < data->nForcePlates; iPlate++)
//    {
//        printf("Force Plate %d\n", data->ForcePlates[iPlate].ID);
//        for(int iChannel=0; iChannel < data->ForcePlates[iPlate].nChannels; iChannel++)
//        {
//            printf("\tChannel %d:\t", iChannel);
//            if(data->ForcePlates[iPlate].ChannelData[iChannel].nFrames == 0)
//            {
//                printf("\tEmpty Frame\n");
//            }
//            else if(data->ForcePlates[iPlate].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
//            {
//                printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->ForcePlates[iPlate].ChannelData[iChannel].nFrames);
//            }
//            for(int iSample=0; iSample < data->ForcePlates[iPlate].ChannelData[iChannel].nFrames; iSample++)
//                printf("%3.2f\t", data->ForcePlates[iPlate].ChannelData[iChannel].Values[iSample]);
//            printf("\n");
//        }
//    }
//
//    // devices
//    printf("Device [Count=%d]\n", data->nDevices);
//    for (int iDevice = 0; iDevice < data->nDevices; iDevice++)
//    {
//        printf("Device %d\n", data->Devices[iDevice].ID);
//        for (int iChannel = 0; iChannel < data->Devices[iDevice].nChannels; iChannel++)
//        {
//            printf("\tChannel %d:\t", iChannel);
//            if (data->Devices[iDevice].ChannelData[iChannel].nFrames == 0)
//            {
//                printf("\tEmpty Frame\n");
//            }
//            else if (data->Devices[iDevice].ChannelData[iChannel].nFrames != g_analogSamplesPerMocapFrame)
//            {
//                printf("\tPartial Frame [Expected:%d   Actual:%d]\n", g_analogSamplesPerMocapFrame, data->Devices[iDevice].ChannelData[iChannel].nFrames);
//            }
//            for (int iSample = 0; iSample < data->Devices[iDevice].ChannelData[iChannel].nFrames; iSample++)
//                printf("%3.2f\t", data->Devices[iDevice].ChannelData[iChannel].Values[iSample]);
//            printf("\n");
//        }
//    }
}

// MessageHandler receives NatNet error/debug messages
void NATNET_CALLCONV NatNetWrapper::MessageHandler( Verbosity msgType, const char* msg )
{

    // Optional: Filter out debug messages
    if ( msgType < verbose_level )
    {
        return;
    }

    printf( "\n[NatNetLib]" );

    switch ( msgType )
    {
        case Verbosity_Debug:
            printf( " [DEBUG]" );
            break;
        case Verbosity_Info:
            printf( "  [INFO]" );
            break;
        case Verbosity_Warning:
            printf( "  [WARN]" );
            break;
        case Verbosity_Error:
            printf( " [ERROR]" );
            break;
        default:
            printf( " [?????]" );
            break;
    }

    printf( ": %s\n", msg );
}

void NATNET_CALLCONV NatNetWrapper::ServerDiscoveredCallback( const sNatNetDiscoveredServer* pDiscoveredServer, void* pUserContext )
{
    char serverHotkey = '.';
    if ( g_discoveredServers.size() < 9 )
    {
        serverHotkey = static_cast<char>('1' + g_discoveredServers.size());
    }

    const char* warning = "";

    if ( pDiscoveredServer->serverDescription.bConnectionInfoValid == false )
    {
        warning = " (WARNING: Legacy server, could not autodetect settings. Auto-connect may not work reliably.)";
    }

    printf( "[%c] %s %d.%d at %s%s\n",
            serverHotkey,
            pDiscoveredServer->serverDescription.szHostApp,
            pDiscoveredServer->serverDescription.HostAppVersion[0],
            pDiscoveredServer->serverDescription.HostAppVersion[1],
            pDiscoveredServer->serverAddress,
            warning );

    g_discoveredServers.push_back( *pDiscoveredServer );
    is_ServerDiscovered = true; //TODO: mutex to protect this
}

// Establish a NatNet Client connection
int NatNetWrapper::ConnectClient()
{
    // Release previous server
    g_pClient->Disconnect();

    // Init Client and connect to NatNet server
    int retCode = g_pClient->Connect( g_connectParams );
    if (retCode != ErrorCode_OK)
    {
        printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
        return ErrorCode_Internal;
    }
    else
    {
        // connection succeeded

        void* pResult;
        int nBytes = 0;
        ErrorCode ret = ErrorCode_OK;

        // print server info
        memset( &g_serverDescription, 0, sizeof( g_serverDescription ) );
        ret = g_pClient->GetServerDescription( &g_serverDescription );
        if ( ret != ErrorCode_OK || ! g_serverDescription.HostPresent )
        {
            printf("Unable to connect to server. Host not present. Exiting.");
            return 1;
        }
        printf("\n[NatNetWrapper] Server application info:\n");
        printf("Application: %s (ver. %d.%d.%d.%d)\n", g_serverDescription.szHostApp, g_serverDescription.HostAppVersion[0],
               g_serverDescription.HostAppVersion[1], g_serverDescription.HostAppVersion[2], g_serverDescription.HostAppVersion[3]);
        printf("NatNet Version: %d.%d.%d.%d\n", g_serverDescription.NatNetVersion[0], g_serverDescription.NatNetVersion[1],
               g_serverDescription.NatNetVersion[2], g_serverDescription.NatNetVersion[3]);
        printf("Client IP:%s\n", g_connectParams.localAddress );
        printf("Server IP:%s\n", g_connectParams.serverAddress );
        printf("Server Name:%s\n", g_serverDescription.szHostComputerName);

        // get mocap frame rate
        ret = g_pClient->SendMessageAndWait("FrameRate", &pResult, &nBytes);
        if (ret == ErrorCode_OK)
        {
            float fRate = *((float*)pResult);
            printf("Mocap Framerate : %3.2f\n", fRate);
        }
        else
            printf("Error getting frame rate.\n");

//        // get # of analog samples per mocap frame of data
//        ret = g_pClient->SendMessageAndWait("AnalogSamplesPerMocapFrame", &pResult, &nBytes);
//        if (ret == ErrorCode_OK)
//        {
//            g_analogSamplesPerMocapFrame = *((int*)pResult);
//            printf("Analog Samples Per Mocap Frame : %d\n", g_analogSamplesPerMocapFrame);
//        }
//        else
//            printf("Error getting Analog frame rate.\n");
    }

    return ErrorCode_OK;
}

void NatNetWrapper::sigintCallback(int signum){
    quit.store(true);
}

void NatNetWrapper::resetClient()
{
    int iSuccess;

    printf("\n\nre-setting Client\n\n.");

    iSuccess = g_pClient->Disconnect();
    if(iSuccess != 0)
        printf("error un-initting Client\n");

    iSuccess = g_pClient->Connect( g_connectParams );
    if(iSuccess != 0)
        printf("error re-initting Client\n");
}

#ifndef _WIN32
char NatNetWrapper::getch()
{
    char buf = 0;
    termios old = { 0 };

    fflush( stdout );

    if ( tcgetattr( 0, &old ) < 0 )
        perror( "tcsetattr()" );

    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;

    if ( tcsetattr( 0, TCSANOW, &old ) < 0 )
        perror( "tcsetattr ICANON" );

    if ( read( 0, &buf, 1 ) < 0 )
        perror( "read()" );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;

    if ( tcsetattr( 0, TCSADRAIN, &old ) < 0 )
        perror( "tcsetattr ~ICANON" );

    //printf( "%c\n", buf );

    return buf;
}

geometry_msgs::PoseStamped NatNetWrapper::rigidBodyToPose(const sRigidBodyData& rigid_body_data) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = rigid_body_data.x;
    pose.pose.position.y = rigid_body_data.y;
    pose.pose.position.z = rigid_body_data.z;
    pose.pose.orientation.x = rigid_body_data.qx;
    pose.pose.orientation.y = rigid_body_data.qy;
    pose.pose.orientation.z = rigid_body_data.qz;
    pose.pose.orientation.w = rigid_body_data.qw;
    return pose;
}

void NatNetWrapper::publishPose(int idx, const sRigidBodyData& rigid_body_data) {
    geometry_msgs::PoseStamped pose = rigidBodyToPose(rigid_body_data);
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = frame_id;
    pubs_vision_pose[idx].publish(pose);
}

void NatNetWrapper::publishOdom(int idx, const sRigidBodyData& rigid_body_data) {
    geometry_msgs::PoseStamped pose = rigidBodyToPose(rigid_body_data);
    nav_msgs::Odometry vision_odom = linearKalmanFilters[idx].get()->pose_cb(pose);
    vision_odom.header.stamp = ros::Time::now();
    vision_odom.header.frame_id = frame_id;
    vision_odom.child_frame_id = frame_id;
    pubs_vision_odom[idx].publish(vision_odom);
}

void NatNetWrapper::publishTF(int idx, const sRigidBodyData& rigid_body_data) {
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(rigid_body_data.x, rigid_body_data.y, rigid_body_data.z));
    tf::Quaternion q(rigid_body_data.qx, rigid_body_data.qy, rigid_body_data.qz, rigid_body_data.qw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, model_names[idx]));
}


#endif
