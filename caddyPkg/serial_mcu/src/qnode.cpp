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
#include "../include/serial_mcu/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace serial_mcu {

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
	ros::init(init_argc,init_argv,"serial_mcu");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    velocity_sub = n.subscribe("/velocity_mcu", 100, &QNode::velocityCallback, this);

    start();
	return true;
}



void QNode::run() {
	ros::Rate loop_rate(1);

    while ( ros::ok() ) {

        ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::velocityCallback(const caddy_msg::wheel_msg &wheelData)
{
    m_wheelData = wheelData;
    std::cout<<"cccccccccccccccccccccccccccccccc"<<std::endl;
    std::cout<<"m_wheelData: "<<m_wheelData<<std::endl;
    std::cout<<"cccccccccccccccccccccccccccccccc"<<std::endl;

    Q_EMIT velocity();
}
}  // namespace serial_mcu
