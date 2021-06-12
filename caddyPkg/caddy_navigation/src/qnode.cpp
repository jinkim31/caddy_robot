/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/caddy_navigation/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace caddy_navigation {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
    init_argc(argc),
    init_argv(argv)
{}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown(); // explicitly needed since we use ros::start();
        ros::waitForShutdown();
    }
    wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"caddy_navigation");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    //   Add your ros communications here.
    imu_sub = n.subscribe("/imu", 100, &QNode::imuCallback, this);
    laser_sub = n.subscribe("/scan", 100, &QNode::laserCallback, this);
    goalpos_sub = n.subscribe("odomPosition", 100, &QNode::odomCallback, this);

    velocity_pub = n.advertise<caddy_msg::wheel_msg>("/velocity", 100);

    start();
    return true;
}


void QNode::run() {
//    ros::Rate loop_rate(33);

//    while ( ros::ok() ) {

//        ros::spinOnce();
//        loop_rate.sleep();
//    }
    ros::spin();


    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::imuCallback(const caddy_msg::imu_msg& imuData)
{
    //    std::cout<<"imuCallback!"<<std::endl;
    m_imuData = imuData;
    Q_EMIT imu();
}

void QNode::odomCallback(const caddy_msg::odomPos& odomPosData)
{
    //    std::cout<<"qnode odomCallback!"<<std::endl;
    m_odomPosData = odomPosData;
    Q_EMIT odompos();
}

void QNode::laserCallback(const sensor_msgs::LaserScan& data)
{
    //    std::cout<<"laserCallback!"<<std::endl;
    m_laserData = data;
    Q_EMIT laser();
}

}  // namespace caddy_navigation
