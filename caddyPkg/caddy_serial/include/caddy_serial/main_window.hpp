/**
 * @file /include/caddy_serial/main_window.hpp
 *
 * @brief Qt based gui for caddy_serial.
 *
 * @date November 2010
 **/
#ifndef caddy_serial_MAIN_WINDOW_H
#define caddy_serial_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/



#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <gps_common/conversions.h>



/*****************************************************************************
** Namespace
*****************************************************************************/

namespace caddy_serial {

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

    double m_GoalX = 0.0;
    double m_GoalY = 0.0;
    double m_RobotX = 0.0;
    double m_RobotY = 0.0;
    double m_Speed = 10.0;


    caddy_msg::remote m_remoteData;
    caddy_msg::wheel_msg m_wheelData;

    sensor_msgs::NavSatFixConstPtr m_goalPos;
    sensor_msgs::NavSatFixConstPtr m_robotPos;
    bool m_goalflag = false;
    bool m_robotflag =false;
    bool m_velflag = false;
    void pubUTMCoordinate(double goalLatitude, double goalLongitude, double robotLatitude, double robotLongitude);
    ~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:

    void on_pb_robot_clicked();
    void on_pb_goal_clicked();
    void on_pb_position_clicked();

    void timerCallback();
    void goalposCallback();
    void robotposCallback();
    void remoteCallback();

    void velocityCallback();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    QTimer* mtimer;

    bool modeFlag = true;

};

}  // namespace caddy_serial

#endif // caddy_serial_MAIN_WINDOW_H
