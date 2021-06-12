/**
 * @file /include/bluetooth_serial/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef bluetooth_serial_QNODE_HPP_
#define bluetooth_serial_QNODE_HPP_

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
#include <caddy_msg/remote.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace bluetooth_serial {

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
    ros::Publisher GpsFixGoal_pub;
    ros::Publisher remote_pub;


Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
};

}  // namespace bluetooth_serial

#endif /* bluetooth_serial_QNODE_HPP_ */
