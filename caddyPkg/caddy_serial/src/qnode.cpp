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
#include "../include/caddy_serial/qnode.hpp"


//int32 speedUp
//int32 speedDown
//int32 go
//int32 left
//int32 right
//int32 back

/*****************************************************************************
** Namespaces
*****************************************************************************/
using namespace std;
namespace caddy_serial {

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

	ros::init(init_argc,init_argv,"caddy_serial");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
    remote_sub   = n.subscribe("remote" , 10, &QNode::remoteCallback, this);


    GpsFixRobot_sub  = n.subscribe("/gps/fix/robot", 10, &QNode::callbackRobot, this);
    GpsFixGoal_sub   = n.subscribe("/gps/fix/goal" , 10, &QNode::callbackGoal, this);
    vleocity_sub = n.subscribe("/velocity", 100, &QNode::velocityCallback, this);

    odomPosition_pub = n.advertise<caddy_msg::odomPos>("/odomPosition", 100);
    velocityMcu_pub = n.advertise<caddy_msg::wheel_msg>("/velocity_mcu", 100);

//    GpsFixRobot_sub = n.subscribe("/gps/fix/robot", 10, callbackRobot); // 바꿈
//    GpsFixGoal_sub  = n.subscribe("/gps/fix/goal", 10, callbackGoal); // 바꿈
//    robotpos_sub = n.subscribe("odomRobot", 100, &QNode::robotPosCallback, this);


//    GpsFixRobot_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix/robot", 100);
//    GpsFixGoal_pub = n.advertise<sensor_msgs::NavSatFix>("/gps/fix/goal", 100);

	start();
	return true;
}

//void QNode::run() {
//    ros::Rate loop_rate(33);

//    while ( ros::ok() ) {

//        ros::spinOnce();
//        loop_rate.sleep();
//    }

//    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
//    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
//}

void QNode::run() {
    ros::spin();
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}
void QNode::remoteCallback(const caddy_msg::remote &data)
{
    //    cout<<" @@@@@@@@@@@@@@@@@@@@@@@@@@@@ remoteCallback @@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    m_remoteData = data;
    Q_EMIT remote();

}

//<sensor_msgs::NavSatFix
void QNode::callbackRobot(const sensor_msgs::NavSatFixConstPtr &fix)
{
//    cout<<" @@@@@@@@@@@@@@@@@@@@@@@@@@@@ callbackRobot @@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    m_robotPos = fix;
    Q_EMIT robotPos();
}
void QNode::callbackGoal(const sensor_msgs::NavSatFixConstPtr& fix)
{
//    cout<<" @@@@@@@@@@@@@@@@@@@@@@@@@@@@ callbackGoal @@@@@@@@@@@@@@@@@@@@@@@@@@@@"<<endl;
    m_goalPos  = fix;
    Q_EMIT goalPos();
}

void QNode::velocityCallback(const caddy_msg::wheel_msg &wheelData)
{
    m_wheelData = wheelData;
    Q_EMIT velocity();
}

}  // namespace caddy_serial
