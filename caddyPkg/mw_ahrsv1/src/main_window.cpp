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
#include "../include/mw_ahrsv1/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace mw_ahrsv1 {

using namespace Qt;
using namespace std;

QByteArray Rx_data;
QByteArray Tx_data;
unsigned char TxBuffer[10] = {0, };


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();
    if(!open_serial())
        exit(0);

    init_mw_ahrsv1();
    past_yaw = 0;



    QObject::connect(serial, SIGNAL(readyRead()), this, SLOT(mw_ahrsv1CallBack()));
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

bool MainWindow::open_serial()
{
    serial = new QSerialPort(this);
    serial -> setPortName("USBIMU");
    serial -> setBaudRate(QSerialPort::Baud115200);
    serial -> setDataBits(QSerialPort::Data8);
    serial -> setParity(QSerialPort::NoParity);
    serial -> setStopBits(QSerialPort::OneStop);
    serial -> setFlowControl(QSerialPort::NoFlowControl);
    if(!serial -> open(QIODevice::ReadWrite))
    {
        cout<<"**********Serial port open fail!!**********"<<endl;
        return false;
    }
    else
        return true;
}


void MainWindow::init_mw_ahrsv1()
{
    TxBuffer[0] = A;     // a
    TxBuffer[1] = N;     // n
    TxBuffer[2] = G;     // g
    TxBuffer[3] = CR;    // CR
    TxBuffer[4] = LF;    // LF
//    TxBuffer[5] = G;
//    TxBuffer[6] = Y;
//    TxBuffer[7] = R;
//    TxBuffer[8] = CR;
//    TxBuffer[9] = LF;
    TxBuffer[5] = M;
    TxBuffer[6] = A;
    TxBuffer[7] = G;
    TxBuffer[8] = CR;
    TxBuffer[9] = LF;


    for(int i = 0; i < 10; i++)
    {
        Tx_data.push_back(TxBuffer[i]);
    }

    serial -> write(Tx_data,10);
    cout<<"TX_DATA ===="<<endl;
    qDebug()<< Tx_data;
}

void MainWindow::mw_ahrsv1CallBack()
{

    if(!serial->isOpen())
    {
        open_serial();
        serial -> write(Tx_data,10);
        cout<<"TX_DATA ===="<<endl;
        qDebug()<< Tx_data;
    }
    //    serial->isWritable()

    if(serial->isReadable())
    {
        Rx_data = serial->readAll();
        char *Rx_Buf = Rx_data.data();
        cout<<"=================================="<<endl<<endl;
        qDebug()<< Rx_data;
        std::cout << std::endl<<endl;

        char *ptr;
        ptr = strtok(Rx_Buf," \n");
        cout<<ptr<<endl;

        //        for(int i = 0; i < 8; i++){
        //            ptr = strtok(Rx_Buf," \n");
        //            cout<<"ptr ="<<ptr<<endl;
        //        }

        if(strcmp(ptr,"ang=   "))
        {

            int i = 0;
            while(ptr != NULL)
            {
//                cout<<"ptr ="<<ptr<<endl;

                if(i == 1)
                    euler.roll = atof(ptr);
                else if(i == 2)
                    euler.pitch = atof(ptr);
                else if(i == 3)
                    euler.yaw = atof(ptr);
                else if(i == 5)
                    mag.x = atof(ptr);
                else if(i == 6)
                    mag.y = atof(ptr);
                else if(i == 7)
                    mag.z = atof(ptr);
                ptr = strtok(NULL, " \n");
                i++;
            }
                euler.yaw -= past_yaw;



                if(euler.yaw > 180) euler.yaw -= 360;
                else if(euler.yaw <= -180)  euler.yaw += 360;

//                ui.lcdNumber->display(euler.yaw);
                ui.lcdNumber2->display(euler.yaw);

                imu_msg.roll = euler.roll;
                imu_msg.pitch = euler.pitch;
                imu_msg.yaw = euler.yaw;
//                imu_msg.roll_acc = euler.roll_acc;
//                imu_msg.pitch_acc = euler.pitch_acc;
//                imu_msg.yaw_acc = euler.yaw_acc;
                imu_msg.x_mag = mag.x;
                imu_msg.y_mag = mag.y;
                imu_msg.z_mag = mag.z;


                std::cout << "roll = " << euler.roll << std::endl;
                std::cout << "pitch = " << euler.pitch << std::endl;
                std::cout << "yaw = " << euler.yaw << std::endl;
//                std::cout << "roll_acc = " << euler.roll_acc << std::endl;
//                std::cout << "pitch_acc = " << euler.pitch_acc << std::endl;
//                std::cout << "yaw_acc = " << euler.yaw_acc << std::endl;
                //        std::cout<<std::endl<<"SSISISISISISISISIBAIIISIBSI"<<std::endl<<std::endl;
                std::cout << std::endl;

                qnode.Imu_pub.publish(imu_msg);
                serial->write(Tx_data,5);
                cout<<"TX_DATA ===="<<endl;
                qDebug()<< Tx_data;
                Rx_data.clear();
        }
        else {
            qnode.Imu_pub.publish(imu_msg);
            serial->write(Tx_data,10);
            cout<<"TX_DATA ===="<<endl;
            qDebug()<< Tx_data;
            Rx_data.clear();
        }

    }
}

void MainWindow::on_pushButton_set_clicked()
{
    cout<<"set"<<endl;
    past_yaw += euler.yaw;
}

MainWindow::~MainWindow() {
    serial->close();
}



}  // namespace mw_ahrsv1

