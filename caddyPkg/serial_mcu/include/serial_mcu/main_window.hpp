/**
 * @file /include/serial_mcu/main_window.hpp
 *
 * @brief Qt based gui for serial_mcu.
 *
 * @date November 2010
 **/
#ifndef serial_mcu_MAIN_WINDOW_H
#define serial_mcu_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
/*****************************************************************************
** Namespace
*****************************************************************************/
using namespace std;
namespace serial_mcu {

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
    QSerialPort *m_serial;
    bool open_serial();

	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
    double m_omega_L = 0;
    double m_omega_R = 0;
    double m_tempOmega_L = 0;
    double m_tempOmega_R = 0;
public Q_SLOTS:
    /********************************************/
    void serialCallBack();
    void on_pb_stop_clicked();
    void velocityCallback();
    void timerCallback();
    void on_pb_right_clicked();
    void on_pb_left_clicked ();
    void on_pb_go_clicked   ();
    void on_pb_back_clicked ();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;

    QTimer* mtimer;

};

}  // namespace serial_mcu

#endif // serial_mcu_MAIN_WINDOW_H
