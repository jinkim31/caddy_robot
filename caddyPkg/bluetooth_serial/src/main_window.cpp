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
#include "../include/bluetooth_serial/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

#define GO          1
#define LEFT        2
#define BACK        3
#define RIGHT       4
#define STOP        5
#define SPEEDUP     6
#define SPEEDDOWN   7
namespace bluetooth_serial {

using namespace Qt;
using namespace std;

QByteArray g_Rx_dataBT;
QByteArray g_Tx_dataBT;
#define check_number 6

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    qnode.init();
    if(!open_serialBT())
        exit(0);
    mTimer = new QTimer(this);
    connect(mTimer, SIGNAL(timeout()),this, SLOT(timerCallback()));
    mTimer->start(500); // 500ms
}

MainWindow::~MainWindow() {}

void MainWindow::timerCallback()
{
    cout<<"modeFlag: "<<modeFlag<<"     m_flagSerialBT: "<<m_flagSerialBT<<endl;
    if(modeFlag == true && m_flagSerialBT == true) // it is following mode
    {
        cout<<" ************************* timerCallback ************************* "<<endl;
        sensor_msgs::NavSatFix data;
        data.latitude  =  m_latitude ;
        data.longitude =  m_longitude;
        qnode.GpsFixGoal_pub.publish(data);

        m_flagSerialBT = false;
    }
}

bool MainWindow::open_serialBT()
{
    //    QString *port = ;
    m_serialBT = new QSerialPort(this);
    //    m_serialBT -> setPortName(ui.cb_BT->currentText());
//    m_serialBT -> setPortName("ESP32_SPP___");
    m_serialBT -> setPortName("/dev/rfcomm0");
    m_serialBT -> setBaudRate(QSerialPort::Baud115200);
    m_serialBT -> setDataBits(QSerialPort::Data8);
    m_serialBT -> setParity(QSerialPort::NoParity);
    m_serialBT -> setStopBits(QSerialPort::OneStop);
    m_serialBT -> setFlowControl(QSerialPort::NoFlowControl);

    if(!m_serialBT -> open(QIODevice::ReadOnly))
    {
        cout<<"**********BT Serial port open fail!!**********"<<endl;
        //        backgroundColourButton->setStyleSheet("background-color: red");
        //        ui.pb_BTOpen->setStyleSheet("background-color: red");
        return false;
    }
    else
    {
        cout<<"**********BT Serial port open success!!**********"<<endl;
        //        ui.pb_BTOpen->setStyleSheet("background-color: rgb(114, 159, 207)");
    }

    QObject::connect(m_serialBT, SIGNAL(readyRead()), this, SLOT(BT_CallBack()));
    return true;

}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::BT_CallBack() // GPS
{
    //    cout<<" ***************************** BT_CallBack ***************************** "<<endl;
    if(m_serialBT->isReadable())
    {
        g_Rx_dataBT = m_serialBT->readAll();
        char *Rx_Buf = g_Rx_dataBT.data();
        double longitude = 0;
        double latitude = 0;
        string dataStr = Rx_Buf;
        static caddy_msg::remote dataRemote;

        static bool startflag = false;
        static string strdata = {NULL, };

        cout<<"dataStr.size: "<<dataStr.size()<<endl;
        cout<<"dataStr: "<<dataStr<<endl;

        // mode control(remote mode or following mode)
        if (dataStr.find("#1") != string::npos) {
            cout << dataStr.find("#1") << endl;
            cout<<"remote mode"<<endl;
            modeFlag = false;
            strdata.clear();
            startflag = false;
        }
        else if (dataStr.find("#0") != string::npos) {
            cout << dataStr.find("#0") << endl;
            cout<<"no remote mode"<<endl;
            modeFlag = true;
            strdata.clear();
            startflag = false;
        }
        else
        {
            // remote mode
            if(modeFlag == false)
            {
                double angR = 0.0;
                double angL = 0.0;
                if(dataStr[0] == '@') // Controll
                {
                    if(dataStr[1] == 'g')
                    {
                        cout<<"GO"<<endl;
                        dataRemote.data = GO;

//                        angR = m_Speed;
//                        angL = m_Speed;
//                        caddy_msg::wheel_msg wheelData;
//                        wheelData.ang_r = angR;
//                        wheelData.ang_l = angL; // publish target angular velocity
//                        cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
//                        cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
////                        qnode.velocity_pub.publish(wheelData);
                    }
                    else if(dataStr[1] == 'l')
                    {
                        cout<<"LEFT"<<endl;
                        dataRemote.data = LEFT;

//                        angR = m_Speed;
//                        angL = -m_Speed;
//                        caddy_msg::wheel_msg wheelData;
//                        wheelData.ang_r = angR;
//                        wheelData.ang_l = angL; // publish target angular velocity
//                        cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
//                        cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
////                        qnode.velocity_pub.publish(wheelData);
                    }
                    else if(dataStr[1] == 'r')
                    {
                        cout<<"RIGHT"<<endl;
                        dataRemote.data = RIGHT;

//                        angR = -m_Speed;
//                        angL = m_Speed;
//                        caddy_msg::wheel_msg wheelData;
//                        wheelData.ang_r = angR;
//                        wheelData.ang_l = angL; // publish target angular velocity
//                        cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
//                        cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
//                        qnode.velocity_pub.publish(wheelData);
                    }
                    else if(dataStr[1] == 'b')
                    {
                        cout<<"BACK"<<endl;                        
                        dataRemote.data = BACK;

//                        angR = -m_Speed;
//                        angL = -m_Speed;
//                        caddy_msg::wheel_msg wheelData;
//                        wheelData.ang_r = angR;
//                        wheelData.ang_l = angL; // publish target angular velocity
//                        cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
//                        cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
////                        qnode.velocity_pub.publish(wheelData);
                    }
                    else if(dataStr[1] == 's')
                    {
                        cout<<"STOP"<<endl;
                        dataRemote.data = STOP;

//                        angR = 0.0;
//                        angL = 0.0;
//                        caddy_msg::wheel_msg wheelData;
//                        wheelData.ang_r = angR;
//                        wheelData.ang_l = angL; // publish target angular velocity
//                        cout<<"wheelData.ang_r: "<<wheelData.ang_r <<endl;
//                        cout<<"wheelData.ang_l: "<<wheelData.ang_l <<endl;
////                        qnode.velocity_pub.publish(wheelData);
                    }
                    else if(dataStr[1] == 'u')
                    {
                        cout<<"Speed Up"<<endl;

                        dataRemote.data = SPEEDUP;
//                        m_Speed +=5;
                    }
                    else if(dataStr[1] == 'd')
                    {
                        cout<<"Speed Down"<<endl;
                        dataRemote.data = SPEEDDOWN;
//                        m_Speed -=5;
                    }
                }

            }
            else if((int)dataStr[0] >= 97 || dataStr.size() >= 25) // alpabet
            {
                strdata.clear();
                startflag = false;
            }
            else // Following mode
            {

                if(dataStr[0]  == '$')
                {
                    strdata.clear();
                    strdata += dataStr;
                    startflag = true;
                    cout<<"start!!"<<endl;
                }
                else if(startflag ==true)
                {
                    strdata +=dataStr;
                    if(strdata.find('*') != string::npos)
                    {
                        std::istringstream ss(strdata);
                        string strWord;
                        vector<string> x;
                        while (getline(ss, strWord, '@'))
                            x.push_back(strWord);

                        if(x.size() >= 2)
                        {
                            // calculate latitude
                            int index = x[0].find(".");
                            string str1 = x[0].substr(1,index - 3);
                            string str2 = x[0].substr(index - 2);
                            double Latitude1 = stod(str1);
                            double Latitude2 = stod (str2) /(double) 60.0;
                            latitude = (Latitude1 + Latitude2);
                            printf("latitude:%.9f \n", latitude);

                            // calculate longitude
                            index = x[1].find(".");
                            str1 = x[1].substr(0,index - 2);
                            str2 = x[1].substr(index - 2);
                            double Longitude1 = stod(str1);
                            double Longitude2 = stod (str2) /(double) 60.0;
                            longitude = (Longitude1 + Longitude2);
                            printf("longitude:%.9f \n", longitude);

                            m_latitude = latitude;
                            m_longitude = longitude;

                            startflag =false;
                        }
                    }
                }
            }
        }
        dataRemote.mode = modeFlag;
        qnode.remote_pub.publish(dataRemote);

    }
    //        cout<<" ********************************** modeFlag: "<<modeFlag<<" ******************************************** "<<endl;

    g_Rx_dataBT.clear();
    m_flagSerialBT = true;
}


//        if(modeFlag == false)
//            ui.le_mode->setText("Remote mode");
//        else
//            ui.le_mode->setText("Following mode");





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

}  // namespace bluetooth_serial

