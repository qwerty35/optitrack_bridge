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
private:
    FMatrix F = FMatrix::Zero();
    GMatrix G = GMatrix::Zero();
    QMatrix Q = QMatrix::Zero();
    RMatrix R = RMatrix::Zero();
    HMatrix H = HMatrix::Zero();

    NVector x_old = NVector::Zero();
    NVector x_predict = NVector::Zero();
    NVector x_estimate = NVector::Zero();
    PMatrix P_old = PMatrix::Zero();
    PMatrix P_predict = PMatrix::Zero();
    PMatrix P_estimate = PMatrix::Zero();

    MVector sigma_Q = MVector::Zero();
    MVector sigma_R = MVector::Zero();

    geometry_msgs::PoseStamped pose_old, pose_new;
    geometry_msgs::TwistStamped twist, twist_raw;
    nav_msgs::Odometry odom;

    ros::Subscriber pose_sub;
    ros::Publisher odom_pub, twist_pub_raw;

    ros::WallTime t_old, t_new;
    bool initialized = false;

    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void predict(const double &dt);
    FMatrix computeF(const double &dt);
    GMatrix computeG(const double &dt);
    QMatrix computeQ(const GMatrix &G, const MVector &sigma_Q);
    void update(const double &dt, const geometry_msgs::PoseStamped::ConstPtr &msg);
    RMatrix computeR();
};


#endif //OPTITRACK_BRIDGE_LINEARKALMANFILTER_H
