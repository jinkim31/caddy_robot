/**
 * @file /include/mw_ahrsv1/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef mw_ahrsv1_QNODE_HPP_
#define mw_ahrsv1_QNODE_HPP_

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
//#include <caddy_msg/imu_msg.h>
#include <caddy_msg/imu_msg.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mw_ahrsv1 {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
        ros::Publisher Imu_pub;
	bool init();
	void run();


Q_SIGNALS:
        void rosShutdown();

private:
	int init_argc;
	char** init_argv;

};

}  // namespace mw_ahrsv1

#endif /* mw_ahrsv1_QNODE_HPP_ */
