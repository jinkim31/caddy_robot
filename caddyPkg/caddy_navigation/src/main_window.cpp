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
#include <iostream>
#include "../include/caddy_navigation/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/




namespace caddy_navigation {

using namespace Qt;
using namespace std;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    qnode.init();
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(laser()), this, SLOT(laserCallback()));
    QObject::connect(&qnode, SIGNAL(odompos()), this, SLOT(odomPosCallback()));
    QObject::connect(&qnode, SIGNAL(imu()), this, SLOT(imudataCallback()));

    m_PF.setPF(KATT, KREP, 1.5, 5);              // k_att, k_rep, rhoZero, kernel size
    m_kinematics.init(0.15,0.15, 0.8, 0.5); // radius_r, radius_l, phi, k_p

    mtimer = new QTimer(this);
    connect(mtimer, SIGNAL(timeout()),this, SLOT(timerCallback()));
    mtimer->start(200); // 200ms

    makePlot();


}

MainWindow::~MainWindow() {}


/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

//double goalX = m_goalPos.pose.pose.position.x - m_robotPos.pose.pose.position.x;
//double goalY = m_goalPos.pose.pose.position.y - m_robotPos.pose.pose.position.y;
//m_PF.calPF(qnode.m_laserData, goalX, goalY); // (lidar, goalpositionX, goal positionY)
//m_PF.calVelocity(m_imuData.yaw); // IMU
//cout<<"m_imuData.yaw: "<<m_imuData.yaw<<endl;
//m_kinematics.calKinematics(m_PF.m_omega, m_PF.m_v);
//caddy_msg::wheel_msg wheelData;
//wheelData.ang_r = m_kinematics.m_wR;
//wheelData.ang_l = m_kinematics.m_wL; // publish target angular velocity
//qnode.velocity_pub.publish(wheelData);
//odomflag = false;
//goalflag = false;

void MainWindow::on_pb_set_clicked()
{
    cout<<"set"<<endl;
    past_yaw += m_imuData.yaw;
}

void MainWindow::timerCallback()
{
    if(m_posCnt++<10)
    {
        odomflag = true;
    }
    else
        odomflag = false;

    cout<<"odomflag: "<<odomflag<<"     laserflag: "<<laserflag<<"        imuflag: "<<imuflag<<endl;

    if(odomflag == true && laserflag == true && imuflag == true)
    {
        cout<<" @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ timerCallback @@@@@@@@@@@@@@@@@@@@@@@@@@@@@ "<<endl;
        cout<<"m_odomPos.robot.pose.pose.position.x: "<<m_odomPos.robot.pose.pose.position.x<<"    m_odomPos.robot.pose.pose.position.y: "<<m_odomPos.robot.pose.pose.position.y<<endl;
        cout<<"m_odomPos.goal.pose.pose.position.x:  "<<m_odomPos.goal.pose.pose.position.x<<"     m_odomPos.goal.pose.pose.position.y : "<<m_odomPos.goal.pose.pose.position.y<<endl;


        double goalX = m_odomPos.goal.pose.pose.position.x - m_odomPos.robot.pose.pose.position.x;
        double goalY = m_odomPos.goal.pose.pose.position.y - m_odomPos.robot.pose.pose.position.y;
        cout<<"goalX: "<<goalX<<"       goalY: "<<goalY<<endl;
        double dist = sqrt(goalX*goalX + goalY*goalY);
        caddy_msg::wheel_msg wheelData;
        cout<<"dist : "<<dist<<endl;
        if(dist > 1.0)
        {
            m_PF.calPF(qnode.m_laserData, goalX, goalY); // (lidar, goalpositionX, goal positionY)
            m_PF.calVelocity(DEG2RAD(m_imuData.yaw + 90)); // IMU // set north direction
            //            m_PF.calVelocity(DEG2RAD((30))); // IMU

            cout<<"m_imuData.yaw: "<<m_imuData.yaw<<endl;
            m_kinematics.calKinematics(m_PF.m_omega, m_PF.m_v);
            cout<<"m_PF.m_omega: "<<m_PF.m_omega<<endl;
            cout<<"m_PF.m_v: "<<m_PF.m_v<<endl;
            wheelData.ang_r = m_kinematics.m_wR;
            wheelData.ang_l = m_kinematics.m_wL; // publish target angular velocity
        }
        else
        {
            wheelData.ang_r = 0.0;
            wheelData.ang_l = 0.0; // publish target angular velocity
        }
        qnode.velocity_pub.publish(wheelData);

        odomflag = false;
        laserflag = false;
        imuflag = false;

        updatePlot();
    }
}

void MainWindow::imudataCallback()
{
    //    cout<<"imudataCallback"<<endl;

    m_imuData = qnode.m_imuData;

//    m_imuData.yaw -= past_yaw;

//    if(m_imuData.yaw > 180) m_imuData.yaw -= 360;
//    else if(m_imuData.yaw <= -180)  m_imuData.yaw += 360;
    ui.lcdNumber->display(m_imuData.yaw);

    imuflag = true;
}


void MainWindow::odomPosCallback()
{
        cout<<"odomPosCallback"<<endl;
    m_odomPos = qnode.m_odomPosData;
    odomflag = true;
    m_posCnt = 0;
}

void MainWindow::laserCallback()
{

    laserflag = true;
    //    cout<<" ******************************* laserCallback ******************************* "<<endl;

    //    cout<<"qnode.m_laserData.ranges.size(): "<<qnode.m_laserData.ranges.size()<<endl;
    //    cout<<"qnode.m_laserData.angle_increment: "<<qnode.m_laserData.angle_increment<<endl;
    //    cout<<"qnode.m_laserData.angle_max: "<<qnode.m_laserData.angle_max<<endl;
    //    cout<<"qnode.m_laserData.angle_min: "<<qnode.m_laserData.angle_min<<endl;
    //    cout<<"qnode.m_laserData.range_max: "<<qnode.m_laserData.range_max<<endl;
    //    cout<<"qnode.m_laserData.range_min: "<<qnode.m_laserData.range_min<<endl;
    //    cout<<"laserCallback"<<endl;

    //    int count = qnode.m_laserData.scan_time / qnode.m_laserData.time_increment;

    //    for(int i = 0; i < count; i++)
    //    {
    //        float degree = RAD2DEG(qnode.m_laserData.angle_min + qnode.m_laserData.angle_increment * i);
    //        if(degree < 0)
    //        {
    //            ROS_INFO(": [%f, %f]", -(180 + degree), qnode.m_laserData.ranges[i]);
    //        }
    //        else
    //        {
    //            ROS_INFO(": [%f, %f]", (180 - degree), qnode.m_laserData.ranges[i]);
    //        }
    //    }


    ////    int count = qnode.m_laserData.scan_time / qnode.m_laserData.time_increment;
    //    //  vector<BeamInfo> dist;

    //    for(int i = 0; i < count; i++)
    //    {
    //        float degree = RAD2DEG(qnode.m_laserData.angle_min + qnode.m_laserData.angle_increment * i);
    //        if(degree < 0)
    //        {
    //            ROS_INFO(": [%f, %f]", (180 + degree), qnode.m_laserData.ranges[i]);
    //        }
    //        else
    //        {
    //            ROS_INFO(": [%f, %f]", -(180 - degree), qnode.m_laserData.ranges[i]);
    //        }
    //    }

}




/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/
void MainWindow::updatePlot()
{
    ui.customPlot->clearPlottables()    ;

    QVector<double> lidarX;
    QVector<double> lidarY;
    QVector<double> minFilterX;
    QVector<double> minFilterY;
    QVector<double> rhoZero;

    for(int i = 0; i< m_PF.m_dist.size(); i++)
    {
        lidarX.push_back(m_PF.m_dist[i].deg);
        lidarY.push_back(m_PF.m_dist[i].dist);
        rhoZero.push_back(m_PF.m_rhoZero);
    }
    for(int i = 0; i<m_PF.minFilter.size(); i++)
    {
        minFilterX.push_back(m_PF.minFilter[i].deg);
        minFilterY.push_back(m_PF.minFilter[i].dist);
    }
    //    cout<<"dataX: "<<dataX.size()<<endl;
    cout<<"minFilterX: "<<minFilterX.size()<<endl;

    double bwidth = 0.45;

    ui.customPlot->setInteraction(QCP::iSelectPlottables, true);
    ui.customPlot->setInteraction(QCP::iSelectItems, true);

    ui.customPlot->setMinimumHeight(500);
    ui.customPlot->setMinimumWidth(600);
    ui.customPlot->xAxis->setRange(-112.0, 112.0);
    ui.customPlot->yAxis->setRange(0, 16.0);

    QCPBars *bars = new QCPBars( ui.customPlot->xAxis,  ui.customPlot->yAxis);
    bars->setSelectable(QCP::stSingleData);
    bars->setData(lidarX, lidarY);
    bars->setBrush(QColor(255, 154, 0));
    bars->setPen(QColor(255, 154, 0));
    bars->setWidthType(QCPBars::wtPlotCoords);
    bars->setWidth(bwidth);
    bars->setName("Lidar data");

    bars = new QCPBars( ui.customPlot->xAxis,  ui.customPlot->yAxis);
    bars->setSelectable(QCP::stSingleData);
    bars->setData(minFilterX, minFilterY);
    bars->setBrush(QColor(250, 0.0, 0.0));
    bars->setPen(QColor(250, 0.0, 0.0));
    bars->setWidthType(QCPBars::wtPlotCoords);
    bars->setWidth(bwidth);
    bars->setName("MinFilter data");

    //    QCPLI

    //    ui.customPlot->addGraph();
    //    ui.customPlot->graph(0)->setBrush(QColor(0, 250, 0, 255));
    //    ui.customPlot->graph(0)->setPen(QColor(0, 250, 0, 255));
    //    ui.customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    //    ui.customPlot->graph(0)->setData(lidarX, rhoZero);
    //    ui.customPlot->
    //    ui.customPlot->graph()->set

    // Set the line chart
    //   QCPGraph *graph = customPlot->addGraph(customPlot->xAxis, customPlot->yAxis);


    // rohZero Ploting
    {
        QCPGraph *line = new QCPGraph( ui.customPlot->xAxis,  ui.customPlot->yAxis);
        line->setName("Latest Transaction Price");
        line->setSelectable(QCP::stSingleData);
        line->setData(lidarX, rhoZero);
        line->setPen(QPen(Qt::green, 4));
        line->setName("rhoZero");
    }



//    bars = new QCPBars( ui.customPlot->xAxis,  ui.customPlot->yAxis);
//    bars->setSelectable(QCP::stSingleData);
//    bars->setBrush(QColor(0, 0.0, 255.0));
//    bars->setPen(QColor(0, 0.0, 255.0));
//    bars->setWidthType(QCPBars::wtPlotCoords);
//    bars->setWidth(bwidth);
//    bars->setName("Virtual Obstacle data");
//    bool aaa = false;

//    if(m_PF.m_virtualFlagL == true)
//    {
//        QVector<double> dataDeg;
//        dataDeg.push_back(m_PF.m_virtualObsL.deg);
//        QVector<double> dataDist;
//        dataDist.push_back(m_PF.m_virtualObsL.dist);
//        bars->setData(dataDeg, dataDist);
//        aaa = true;
////        disconnect()
////        disconnect(mtimer, SIGNAL(timeout()),this, SLOT(timerCallback()));

//    }
//    if(m_PF.m_virtualFlagR == true)
//    {
//        QVector<double> dataDeg;
//        dataDeg.push_back(m_PF.m_virtualObsR.deg);
//        QVector<double> dataDist;
//        dataDist.push_back(m_PF.m_virtualObsR.dist);
//        bars->setData(dataDeg, dataDist);
//        aaa =true;
////        disconnect(mtimer, SIGNAL(timeout()),this, SLOT(timerCallback()));
//    }


    ui.customPlot->legend->setVisible(true);
    //    ui.customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignHCenter);
    ui.customPlot->legend->setBrush(QColor(255, 255, 255, 100));
    ui.customPlot->legend->setBorderPen(Qt::NoPen);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    ui.customPlot->legend->setFont(legendFont);
    ui.customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui.customPlot->xAxis->setLabel("Angle(deg)");
    ui.customPlot->xAxis->setLabelColor(Qt::white);
    ui.customPlot->xAxis->setTickLabelColor(Qt::white);

    ui.customPlot->yAxis->setLabel("Dist(m)");
    ui.customPlot->yAxis->setLabelColor(Qt::white);
    ui.customPlot->yAxis->setTickLabelColor(Qt::white);


    ui.customPlot->replot();
    if(m_PF.m_virtualFlagL==true)
    {

        cout<<"m_PF.m_virtualObsL.deg: "<<m_PF.m_virtualObsL.deg<<endl;
        cout<<"m_PF.m_virtualObsL.dist: "<<m_PF.m_virtualObsL.dist<<endl;
        cout<<"m_PF.m_virtualObsR.deg: "<<m_PF.m_virtualObsR.deg<<endl;
        cout<<"m_PF.m_virtualObsR.dist: "<<m_PF.m_virtualObsR.dist<<endl;
//        disconnect(mtimer, SIGNAL(timeout()),this, SLOT(timerCallback()));
        ui.customPlot->savePng("obs");

    }
//        ui.customPlot->savePng("obs", 500,600);




}



void MainWindow::makePlot()
{

    // set dark background gradient:
    {
        QLinearGradient gradient(0, 0, 0, 400);
        gradient.setColorAt(0, QColor(90, 90, 90));
        gradient.setColorAt(0.38, QColor(105, 105, 105));
        gradient.setColorAt(1, QColor(70, 70, 70));
        ui.customPlot->setBackground(QBrush(gradient));
        ui.customPlot->setInteraction(QCP::iSelectPlottables, true);
        ui.customPlot->setInteraction(QCP::iSelectItems, true);
    }
    // set Plot size
    {
        ui.customPlot->setMinimumHeight(500);
        ui.customPlot->setMinimumWidth(600);
        ui.customPlot->xAxis->setRange(-112.0, 112.0);
        ui.customPlot->yAxis->setRange(0, 16.0);
    }

    ui.customPlot->legend->setVisible(true);
//    ui.customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop|Qt::AlignHCenter);
    ui.customPlot->legend->setBrush(QColor(255, 255, 255, 100));
    ui.customPlot->legend->setBorderPen(Qt::NoPen);
    QFont legendFont = font();
    legendFont.setPointSize(10);
    ui.customPlot->legend->setFont(legendFont);
    ui.customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui.customPlot->replot();
    ui.customPlot->xAxis->setLabel("Angle(deg)");
    ui.customPlot->xAxis->setLabelColor(Qt::white);
    ui.customPlot->xAxis->setTickLabelColor(Qt::white);

    ui.customPlot->yAxis->setLabel("Dist(m)");
    ui.customPlot->yAxis->setLabelColor(Qt::white);
    ui.customPlot->yAxis->setTickLabelColor(Qt::white);

}



void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace caddy_navigation

