/**
 * @file /include/caddy_navigation/main_window.hpp
 *
 * @brief Qt based gui for caddy_navigation.
 *
 * @date November 2010
 **/
#ifndef caddy_navigation_MAIN_WINDOW_H
#define caddy_navigation_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "potential_field.h"
#include "mobile_kinematics.h"
#include "qcustomplot.hpp"
#include <caddy_msg/odomPos.h>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace caddy_navigation {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = 0);
    ~MainWindow();
    sensor_msgs::LaserScan m_lidarData;
    potential_field m_PF;
    mobile_kinematics m_kinematics;
    typedef struct
    {
        double x;
        double y;
    }point2d;

    caddy_msg::odomPos m_odomPos;
    caddy_msg::imu_msg m_imuData;

    int m_posCnt = 0;

    float past_yaw;


    bool odomflag =false;
    bool imuflag =false;
    bool laserflag = false;

    void closeEvent(QCloseEvent *event); // Overloaded function
    void makePlot();
    void updatePlot();

public Q_SLOTS:
    /********************************************/
    void timerCallback();

    void laserCallback();
    void odomPosCallback();
    void imudataCallback();
    void on_pb_set_clicked();


private:
    Ui::MainWindowDesign ui;
    QNode qnode;
    QTimer* mtimer;

};

}  // namespace caddy_navigation

#endif // caddy_navigation_MAIN_WINDOW_H
