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
#include "../include/gps_serial/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace gps_serial {

using namespace Qt;
using namespace std;

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
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    open_serialGPS();

}

MainWindow::~MainWindow() {}
bool MainWindow::open_serialGPS()
{
    cout<<"qnode.serial_port.c_str():-"<<qnode.serial_port.c_str()<<"a"<<endl;
    //    string port = .toStdString();
    m_serialGPS = new QSerialPort(this);
    m_serialGPS -> setPortName(QString(qnode.serial_port.c_str()));
    m_serialGPS -> setBaudRate(QSerialPort::Baud9600);
    m_serialGPS -> setDataBits(QSerialPort::Data8);
    m_serialGPS -> setParity(QSerialPort::NoParity);
    m_serialGPS -> setStopBits(QSerialPort::OneStop);
    m_serialGPS -> setFlowControl(QSerialPort::NoFlowControl);
    if(!m_serialGPS -> open(QIODevice::ReadOnly))
    {
        cout<<"**********GPS Serial port open fail!!**********"<<endl;
        //        ui.pb_gpsOpen->setStyleSheet("background-color: red");
        return false;
    }
    else
    {
        cout<<"**********GPS Serial port open succes!!**********"<<endl;
        //        ui.pb_gpsOpen->setStyleSheet("background-color: rgb(114, 159, 207)");

    }
    QObject::connect(m_serialGPS, SIGNAL(readyRead()), this, SLOT(GPS_CallBack()));

    m_flagSerialGPS = true;
    return true;
}

// It is GLONASS - GNGGA data
void MainWindow::calDDMM_MM(string data, double &latitude, double &longitude, int &status) // GPS low data ddmm.mm(도분) to coordinate(latitude, longitude)
{
    std::istringstream ss(data);
    string strWord;
    vector<string> x;

    while (getline(ss, strWord, ','))
        x.push_back(strWord);

    if(x.size() >3)
    {
        //        cout<<"x[0]: "<<x[0]<<endl;
        for(int i = 0; i < x.size(); i++)
        {
            if("$GNGGA" == x[0])
            {
                //                cout<<endl<<" --------------------- GNGGA --------------------- "<<endl;
                //                cout<<"x[2]: "<<x[2]<<endl;
                //                cout<<"x[4]: "<<x[4]<<endl;
                int index = x[2].find(".");
                if(index >= 2)
                {
                    string str1 = x[2].substr(0,index - 2);
                    string str2 = x[2].substr(index - 2);
                    double Latitude1 = stod(str1);
                    double Latitude2 = stod (str2) /(double) 60.0;
                    latitude = (Latitude1 + Latitude2);
                }

                index = x[4].find(".");
                if(index >= 2)
                {
                    string str1 = x[4].substr(0,index - 2);
                    string str2 = x[4].substr(index - 2);
                    double Longitude1 = stod(str1);
                    double Longitude2 = stod (str2) / (double)60.0;
                    longitude = (Longitude1 + Longitude2);
                }
                string fixData = x[6];
                //                cout<<"fixData: "<<fixData<<endl;
                status = stoi(fixData);
                if(status == 1)
                    cout<<"GPS data"<<endl;
                else if(status == 2)
                    cout<<"Differential GPS data(DGPS)"<<endl;
                break;
            }
        }
    }

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::GPS_CallBack() // Robot
{
    //    modeFlag == true;
    //    cout<<"GPS_CallBack"<<endl;
    //    cout<<" ***************************** GPS_CallBack ***************************** "<<endl;

    if(m_serialGPS->isReadable() && modeFlag == true)
    {
        g_Rx_dataGPS = m_serialGPS->readAll();
        char *Rx_Buf = g_Rx_dataGPS.data();
        double longitude = 0;
        double latitude = 0;
        int status = -1;

        //        printf(Rx_Buf);
        //        if(ui.check_SaveAll->isChecked() || ui.check_SaveGPS->isChecked())
        //        {
        //            //            ofstream writeFile("../GPS data/GPS_data_robot.txt", ios::app);
        //            ofstream writeFile("/home/kdh/catkin_ws/src/caddy_robot/caddy_serial/GPS data/GPS_data_robot.txt", ios::app);

        //            writeFile << fixed;
        //            writeFile.precision(5);
        //            writeFile <<Rx_Buf;
        //            writeFile.close();
        //        }
        //         GPS data merge (GPS데이터를 받을 때 데이터가 짤림)
        static bool startflag = false;
        static string strdata = {NULL, };
        // NMEA data - '$' is start
        if(Rx_Buf[0]  == '$')
        {
            strdata.clear();
            strdata += Rx_Buf;
            //                  cout<<" ------------------------------------------------------- start!"<<endl;
            startflag = true;
        }
        else
        {
            if(startflag ==true)
            {
                strdata +=Rx_Buf;
                for(int i = 0; i < sizeof(Rx_Buf); i++)
                {
                    if(Rx_Buf[i] == '*')
                    {
                        int index = strdata.find('*');
                        strdata.resize(index+3);
                        //                                    cout<<"data: "<<strdata<<endl;
                        //            cout<<" ------------------------------------------------------- finish~~~"<<endl;
                        calDDMM_MM(strdata, latitude, longitude, status); //
                        if(latitude != 0.0 &&longitude != 0.0)
                        {
                            printf("GPS latitude: %.9f\n",latitude);
                            printf("GPS longitude: %.9f\n",longitude);
                            sensor_msgs::NavSatFix data;
                            data.latitude = latitude;
                            data.longitude = longitude;
                            //                            sensor_msgs::NavSatStatus::STATUS_NO_FIX;
                            //                            STATUS_NO_FIX = -1,   // no Fix
                            //                            STATUS_FIX = 0,       // fix
                            //                            STATUS_SBAS_FIX = 1,  //
                            //                            STATUS_GBAS_FIX = 2,  // dgps
                            data.status.status = status;
                            qnode.GpsFixRobot_pub.publish(data);

                        }
                        startflag= false;

                    }
                }
            }
        }

        g_Rx_dataGPS.clear();
    }
}


/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/



void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}  // namespace gps_serial

