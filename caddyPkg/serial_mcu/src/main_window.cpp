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
#include "../include/serial_mcu/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace serial_mcu {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

QByteArray g_Rx_data;
QByteArray g_Tx_data;
unsigned char g_TxBuffer[128] = { 0, };
unsigned char g_RxBuffer[10] = { 0, };
unsigned char g_Parameter[128] = { 0, };

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    qnode.init();

    QObject::connect(&qnode, SIGNAL(velocity()), this, SLOT(velocityCallback()));
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

    if(!open_serial())
        exit(0);

    mtimer = new QTimer(this);
    connect(mtimer, SIGNAL(timeout()),this, SLOT(timerCallback()));
    mtimer->start(1000);

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/
void MainWindow::timerCallback()
{
    if(ui.cb_mode->isChecked())
    {
        cout<<"#### timerCallback ####"<<endl<<endl;

        bool Lnegative = m_omega_L < 0 ? true : false;
        bool Rnegative = m_omega_R < 0 ? true : false;

        cout<<" m_omega_L: "<< m_omega_L<<endl;
        cout<<" m_omega_R: "<< m_omega_R<<endl;

        int32_t omegaL = abs(m_omega_L * 10);
        int32_t omegaR = abs(m_omega_R * 10);
        cout<<"m_omega_L: "<<m_omega_L<<"       m_omega_R: "<<m_omega_R<<endl;

        unsigned char PacketLength = 10;
        // Header
        g_TxBuffer[0] = 0xAA;
        g_TxBuffer[1] = 0xBB;
        g_TxBuffer[2] = 1;
        g_TxBuffer[3] = Lnegative == true  ? 1 : 0;
        g_TxBuffer[4] = (omegaL & 0x00FF);
        g_TxBuffer[5] = (omegaL >> 8);
        g_TxBuffer[6] = 2;
        g_TxBuffer[7] = Rnegative == true ? 1 : 0;
        g_TxBuffer[8] = (omegaR & 0x00FF);
        g_TxBuffer[9] = (omegaR >> 8);

        for(int i = 0; i < PacketLength; i++)
        {
            g_Tx_data.push_back(g_TxBuffer[i]);
        }

        for(int i = 0; i < PacketLength;i++)
            printf("TxBuffer[%d] = %d\n",i,g_TxBuffer[i]);

        m_serial -> write(g_Tx_data, PacketLength);

        g_Tx_data.clear();
        m_tempOmega_L = m_omega_L;
        m_tempOmega_R = m_omega_R;
    }

}
bool MainWindow::open_serial()
{
  m_serial = new QSerialPort(this);
  m_serial -> setPortName("USBSERIAL");
  m_serial -> setBaudRate(QSerialPort::Baud115200);
  m_serial -> setDataBits(QSerialPort::Data8);
  m_serial -> setParity(QSerialPort::NoParity);
  m_serial -> setStopBits(QSerialPort::OneStop);
  m_serial -> setFlowControl(QSerialPort::NoFlowControl);
  if(!m_serial -> open(QIODevice::ReadWrite))
  {
    cout<<"**********Serial port open fail!!**********"<<endl;
    return false;
  }
  QObject::connect(m_serial, SIGNAL(readyRead()), this, SLOT(serialCallBack()));

  return true;
}

void MainWindow::serialCallBack()
{
    cout<<"#### serialCallBack ####"<<endl<<endl;

}

void MainWindow::on_pb_right_clicked()
{
    double speed = ui.le_speed->text().toDouble();
    m_omega_L = speed;
    m_omega_R = -speed;
}
void MainWindow::on_pb_left_clicked()
{
    double speed = ui.le_speed->text().toDouble();
    m_omega_L = -speed;
    m_omega_R = speed;
}
void MainWindow::on_pb_go_clicked()
{
    double speed = ui.le_speed->text().toDouble();
    m_omega_L = speed;
    m_omega_R = speed;
}
void MainWindow::on_pb_back_clicked()
{
    double speed = ui.le_speed->text().toDouble();
    m_omega_L = -speed;
    m_omega_R = -speed;
}
void MainWindow::on_pb_stop_clicked()
{
    m_omega_L = 0.0;
    m_omega_R = 0.0;
}

void MainWindow::velocityCallback()
{

    cout<<"#### velocityCallback ####"<<endl<<endl;

    bool Lnegative = qnode.m_wheelData.ang_l < 0 ? true : false;
    bool Rnegative = qnode.m_wheelData.ang_r < 0 ? true : false;

    cout<<" qnode.m_wheelData.ang_l: "<< qnode.m_wheelData.ang_l<<endl;
    cout<<" qnode.m_wheelData.ang_r: "<< qnode.m_wheelData.ang_r<<endl;

    int32_t omegaL = abs(qnode.m_wheelData.ang_l*10);
    int32_t omegaR = abs(qnode.m_wheelData.ang_r*10);

    unsigned char PacketLength = 10;
    // Header
    g_TxBuffer[0] = 0xAA;
    g_TxBuffer[1] = 0xBB;
    g_TxBuffer[2] = 1;
    g_TxBuffer[3] = Lnegative == true  ? 1 : 0;
    g_TxBuffer[4] = (omegaL & 0x00FF);
    g_TxBuffer[5] = (omegaL >> 8);
    g_TxBuffer[6] = 2;
    g_TxBuffer[7] = Rnegative == true ? 1 : 0;
    g_TxBuffer[8] = (omegaR & 0x00FF);
    g_TxBuffer[9] = (omegaR >> 8);

    for(int i = 0; i < PacketLength; i++)
    {
        g_Tx_data.push_back(g_TxBuffer[i]);
    }

    for(int i = 0; i < PacketLength;i++)
        printf("TxBuffer[%d] = %d\n",i,g_TxBuffer[i]);

    m_serial -> write(g_Tx_data, PacketLength);

    g_Tx_data.clear();
    m_tempOmega_L = qnode.m_wheelData.ang_l;
    m_tempOmega_R = qnode.m_wheelData.ang_r;

}
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

}  // namespace serial_mcu

