/**
 * @file /include/caddy_navigation/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef caddy_navigation_QNODE_HPP_
#define caddy_navigation_QNODE_HPP_

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
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
//#include <gps_common/conversions.h>
#include <nav_msgs/Odometry.h>

#include <caddy_msg/wheel_msg.h>
#include <caddy_msg/imu_msg.h>
#include <caddy_msg/odomPos.h>
#include <iostream>

//#include <msg_generate/imu_msg.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace caddy_navigation {
using namespace std;
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
    ros::Subscriber imu_sub;
    ros::Subscriber laser_sub;
    ros::Subscriber goalpos_sub;
    ros::Subscriber robotpos_sub;
    ros::Publisher velocity_pub;
//    void imuCallback(const sensor_msgs::LaserScan& data);
    void laserCallback(const sensor_msgs::LaserScan& data);
    void odomCallback(const caddy_msg::odomPos& odomPosData);
    void imuCallback(const caddy_msg::imu_msg& imuData);

    sensor_msgs::LaserScan m_laserData;
    caddy_msg::odomPos m_odomPosData;
    caddy_msg::wheel_msg m_wheelData;
    caddy_msg::imu_msg m_imuData;

Q_SIGNALS:
    void rosShutdown();

    void laser();
    void odompos();
    void imu();

private:
    int init_argc;
    char** init_argv;
};

}  // namespace caddy_navigation

#endif /* caddy_navigation_QNODE_HPP_ */
