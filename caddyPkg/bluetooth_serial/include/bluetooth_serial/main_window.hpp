/**
 * @file /include/bluetooth_serial/main_window.hpp
 *
 * @brief Qt based gui for bluetooth_serial.
 *
 * @date November 2010
 **/
#ifndef bluetooth_serial_MAIN_WINDOW_H
#define bluetooth_serial_MAIN_WINDOW_H

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

namespace bluetooth_serial {

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
    QSerialPort *m_serialBT;
    bool open_serialBT();
    bool m_flagSerialBT = false;
    double m_Speed = 10.0;
    double  m_latitude;
    double  m_longitude;

	void closeEvent(QCloseEvent *event); // Overloaded function

public Q_SLOTS:
    void BT_CallBack();
    void timerCallback();



private:
	Ui::MainWindowDesign ui;
    QNode qnode;
    QTimer* mTimer;

    bool modeFlag = true;

};

}  // namespace bluetooth_serial

#endif // bluetooth_serial_MAIN_WINDOW_H
