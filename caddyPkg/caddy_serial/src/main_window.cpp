/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include "../include/caddy_serial/main_window.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
/*****************************************************************************
** Namespaces
*****************************************************************************/


namespace caddy_serial {


using namespace Qt;
using namespace std;

QByteArray g_Rx_dataBT;
QByteArray g_Tx_dataBT;
QByteArray g_Rx_dataGPS;
QByteArray g_Tx_dataGPS;
#define check_number 6

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(robotPos()), this, SLOT(robotposCallback()));
    QObject::connect(&qnode, SIGNAL(goalPos ()), this, SLOT(goalposCallback ()));
    QObject::connect(&qnode, SIGNAL(velocity ()), this, SLOT(velocityCallback ()));
    QObject::connect(&qnode, SIGNAL(remote()), this, SLOT(remoteCallback ()));




    mtimer = new QTimer(this);
    connect(mtimer, SIGNAL(timeout()),this, SLOT(timerCallback()));
    mtimer->start(100); // 100ms

}

MainWindow::~MainWindow() {}


// sensor_msgs::NavSatStatus::STATUS_NO_FIX;
// STATUS_NO_FIX = -1,   // no Fix
// STATUS_FIX = 0,       // fix
// STATUS_SBAS_FIX = 1,  //
// STATUS_GBAS_FIX = 2,  // dgps

void MainWindow::pubUTMCoordinate(double goalLatitude, double goalLongitude, double robotLatitude, double robotLongitude)
{
    double northing, easting;
    std::string zone;

    //    if (m_goalPos->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {  test
    //        ROS_DEBUG_THROTTLE(60,"No fix.");
    //        return;
    //    }

    gps_common::LLtoUTM(goalLatitude, goalLongitude, northing, easting, zone);
    nav_msgs::Odometry odomGoal;
//    odomGoal.header.stamp = m_robotPos->header.stamp;  test
    odomGoal.pose.pose.position.x = easting;
    odomGoal.pose.pose.position.y = northing;
//    odomGoal.pose.pose.position.z = m_robotPos->altitude;  test
    ROS_INFO("Goal position X: %f / Goal position Y: %f",  odomGoal.pose.pose.position.x,  odomGoal.pose.pose.position.y);


//    if (m_robotPos->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {  test
//        ROS_DEBUG_THROTTLE(60,"No fix.");
//        return;
//    }

    gps_common::LLtoUTM(robotLatitude, robotLongitude, northing, easting, zone);
    nav_msgs::Odometry odomRobot;
//    odomRobot.header.stamp = m_goalPos->header.stamp;  test
    odomRobot.pose.pose.position.x = easting;
    odomRobot.pose.pose.position.y = northing;
//    odomRobot.pose.pose.position.z = m_goalPos->altitude;  test
    ROS_INFO("Robot position X: %f / Robot position Y: %f",  odomRobot.pose.pose.position.x,  odomRobot.pose.pose.position.y);

    caddy_msg::odomPos odom;
    odom.goal  = odomGoal;
    odom.robot = odomRobot;
    qnode.odomPosition_pub.publish(odom);
}

void MainWindow::timerCallback()
{
    if(ui.cb_testmode->isChecked())
    {
        static int cnt = 0;
        if(cnt++ % 10 == 0)
        {
            cnt= 0;
            double goalLatitude   = ui.le_latitudeGoal  ->text().toDouble();
            double goalLongitude  = ui.le_longitudeGoal ->text().toDouble();
            double robotLatitude  = ui.le_latitudeRobot ->text().toDouble();
            double robotLongitude = ui.le_longitudeRobot->text().toDouble();
            cout<<"aaa"<<endl;
            pubUTMCoordinate(goalLatitude,goalLongitude, robotLatitude, robotLongitude);
            cout<<"bbb"<<endl;
            if(m_velflag == true)
            {
                qnode.velocityMcu_pub.publish(m_wheelData);
                m_velflag = false;
            }
        }

    }
    else
    {
        if(modeFlag == false) // remote mode
        {
            ui.le_mode->setText("Remote mode");
            ui.le_mode->setStyleSheet("background-color: rgb(219, 86, 86);");
        }
        else // following mode
        {
            ui.le_mode->setText("Follwing mode");
            ui.le_mode->setStyleSheet("background-color: rgb(114, 159, 207);");

//            cout<<" ********************* timerCallback ********************* "<<endl;
//            cout<<"m_velflag: "<<m_velflag<<endl;
            cout<<"m_goalflag: "<<m_goalflag<<"     m_robotflag: "<<m_robotflag<<endl;
            if(m_velflag == true) // navigation omega data
            {
                qnode.velocityMcu_pub.publish(m_wheelData);
                m_velflag = false;
            }
            if(m_goalflag == true && m_robotflag ==true)
            {
                pubUTMCoordinate(m_goalPos->latitude,m_goalPos->longitude,m_robotPos->latitude,m_robotPos->longitude);
                m_goalflag  = false;
                m_robotflag = false;
            }
        }

    }


}

void MainWindow::goalposCallback()
{
        cout<<" ********************* goalposCallback ********************* "<<endl;
    m_goalPos = qnode.m_goalPos;
    m_goalflag = true;
}

void MainWindow::robotposCallback()
{
    //    cout<<" ********************* robotposCallback ********************* "<<endl;
    m_robotPos = qnode.m_robotPos;
    m_robotflag = true;
}

void MainWindow::velocityCallback()
{
    m_wheelData = qnode.m_wheelData;
    m_velflag = true;
}

void MainWindow::remoteCallback()
{
    //    cout<<" ********************* remoteCallback ********************* "<<endl;
    m_remoteData = qnode.m_remoteData;
    modeFlag = m_remoteData.mode;
    int dir = m_remoteData.data;

    cout<<"modeFlag: "<<modeFlag<<endl;
    cout<<"dir: "<<dir<<endl;

    // remote mode
    if(modeFlag == false && m_remoteData.data != 0.0)
    {
        double angR = 0.0;
        double angL = 0.0;
        caddy_msg::wheel_msg wheelData;

        switch (dir)
        {
        case 1:
        {
            cout<<"GO"<<endl;

            angR = m_Speed;
            angL = m_Speed;
            wheelData.ang_r = angR;
            wheelData.ang_l = angL; // publish target angular velocity
            cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
            cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
            break;
        }
        case 2:
        {
            cout<<"LEFT"<<endl;

            angR = m_Speed;
            angL = -m_Speed;
            wheelData.ang_r = angR;
            wheelData.ang_l = angL; // publish target angular velocity
            cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
            cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
            break;
        }

        case 3:
        {
            cout<<"BACK"<<endl;

            angR = -m_Speed;
            angL = -m_Speed;
            wheelData.ang_r = angR;
            wheelData.ang_l = angL; // publish target angular velocity
            cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
            cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
            break;
        }

        case 4:
        {
            cout<<"RIGHT"<<endl;

            angR = -m_Speed;
            angL = m_Speed;
            wheelData.ang_r = angR;
            wheelData.ang_l = angL; // publish target angular velocity
            cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
            cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
            break;
        }

        case 5:
        {
            cout<<"STOP"<<endl;
            angR = 0.0;
            angL = 0.0;
            wheelData.ang_r = angR;
            wheelData.ang_l = angL; // publish target angular velocity
            cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
            cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
            break;
        }

        case 6:
        {
            cout<<"SPEED UP"<<endl;
            m_Speed +=5;
            break;
        }

        case 7:
        {
            cout<<"SPEED DOWN"<<endl;
            m_Speed -=5;
            break;
        }

        }

        qnode.velocityMcu_pub.publish(wheelData);

    }
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::on_pb_robot_clicked()
{
    double longitude = ui.le_longitudeRobot->text().toDouble();
    double latitude = ui.le_latitudeRobot->text().toDouble();
    sensor_msgs::NavSatFix data;
    data.latitude = latitude;
    data.longitude = longitude;
    //    qnode.GpsFixRobot_pub.publish(data);

}
void MainWindow::on_pb_goal_clicked()
{
    double longitude = ui.le_longitudeGoal->text().toDouble();
    double latitude = ui.le_latitudeGoal->text().toDouble();
    sensor_msgs::NavSatFix data;
    data.latitude = latitude;
    data.longitude = longitude;
    //    qnode.GpsFixGoal_pub.publish(data);
}
void MainWindow::on_pb_position_clicked()
{
    m_GoalX= ui.le_GoalX->text().toDouble();
    m_GoalY= ui.le_GoalY->text().toDouble();
    m_RobotX = ui.le_RobotX->text().toDouble();
    m_RobotY = ui.le_RobotY->text().toDouble();
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace caddy_serial

