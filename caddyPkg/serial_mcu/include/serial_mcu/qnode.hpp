/**
 * @file /include/serial_mcu/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef serial_mcu_QNODE_HPP_
#define serial_mcu_QNODE_HPP_

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
//#include "/home/kdh/catkin_ws/devel/include/caddy_msg/wheel_msg.h"
#include <caddy_msg/wheel_msg.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace serial_mcu {

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
    ros::Subscriber velocity_sub;
    caddy_msg::wheel_msg m_wheelData;
    void velocityCallback(const caddy_msg::wheel_msg& wheelData);


Q_SIGNALS:
    void rosShutdown();
    void velocity();

private:
	int init_argc;
	char** init_argv;

};

}  // namespace serial_mcu

#endif /* serial_mcu_QNODE_HPP_ */
