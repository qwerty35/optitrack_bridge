#ifndef OPTITRACK_BRIDGE_LINEARKALMANFILTER_H
#define OPTITRACK_BRIDGE_LINEARKALMANFILTER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <string>

#define PUBLISH_RAW_TWIST false

#define n 6
#define m 3

using namespace Eigen;
typedef Matrix<double, n, n> FMatrix;
typedef Matrix<double, n, m> GMatrix;
typedef Matrix<double, n, n> QMatrix;
typedef Matrix<double, n, n> PMatrix;
typedef Matrix<double, m, m> RMatrix;
typedef Matrix<double, m, n> HMatrix;
typedef Matrix<double, n, 1> NVector;
typedef Matrix<double, m, 1> MVector;

class LinearKalmanFilter {
public:
    LinearKalmanFilter();
    nav_msgs::Odometry pose_cb(const geometry_msgs::PoseStamped& msg);

private:
    FMatrix F;
    GMatrix G;
    QMatrix Q;
//    RMatrix R;
    HMatrix H;

    NVector x_old;
    NVector x_predict;
    NVector x_estimate;
    PMatrix P_old;
    PMatrix P_predict;
    PMatrix P_estimate;

    MVector sigma_Q;
    MVector sigma_R;

    geometry_msgs::PoseStamped pose_old, pose_new;
    geometry_msgs::TwistStamped twist, twist_raw;
    nav_msgs::Odometry odom;

    ros::WallTime t_old, t_new;
    bool initialized = false;

    void predict(const double &dt);
    static FMatrix computeF(const double &dt);
    static GMatrix computeG(const double &dt);
    static QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q);
    void update(const double &dt, const geometry_msgs::PoseStamped& msg);
    RMatrix computeR();
};


#endif //OPTITRACK_BRIDGE_LINEARKALMANFILTER_H
