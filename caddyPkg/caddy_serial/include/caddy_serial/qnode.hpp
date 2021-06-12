/**
 * @file /include/caddy_serial/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef caddy_serial_QNODE_HPP_
#define caddy_serial_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <caddy_msg/wheel_msg.h>
#include <caddy_msg/odomPos.h>
#include <caddy_msg/remote.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace caddy_serial {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();

    ros::Publisher velocity_pub;
    ros::Publisher velocityMcu_pub;

    ros::Subscriber remote_sub;
    void remoteCallback(const caddy_msg::remote& data);
    caddy_msg::remote m_remoteData;

    ros::Subscriber GpsFixRobot_sub ;
    ros::Subscriber GpsFixGoal_sub  ;
    ros::Publisher  odomPosition_pub ;
    void callbackRobot(const sensor_msgs::NavSatFixConstPtr &fix);
    void callbackGoal(const sensor_msgs::NavSatFixConstPtr& fix);
    sensor_msgs::NavSatFixConstPtr m_robotPos;
    sensor_msgs::NavSatFixConstPtr m_goalPos;


    void velocityCallback(const caddy_msg::wheel_msg& wheelData);
    caddy_msg::wheel_msg m_wheelData;
    ros::Subscriber  vleocity_sub ;

Q_SIGNALS:
    void rosShutdown();
    void robotPos();
    void goalPos();
    void velocity();
    void remote();


private:
	int init_argc;
	char** init_argv;
};

}  // namespace caddy_serial

#endif /* caddy_serial_QNODE_HPP_ */
