/**
 * @file /include/gps_serial/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gps_serial_QNODE_HPP_
#define gps_serial_QNODE_HPP_

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
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gps_serial {

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
    ros::Publisher GpsFixRobot_pub;
    std::string serial_port;



Q_SIGNALS:
    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
};

}  // namespace gps_serial

#endif /* gps_serial_QNODE_HPP_ */
