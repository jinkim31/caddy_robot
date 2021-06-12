/**
 * @file /include/gps_serial/main_window.hpp
 *
 * @brief Qt based gui for gps_serial.
 *
 * @date November 2010
 **/
#ifndef gps_serial_MAIN_WINDOW_H
#define gps_serial_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
//#include <gps_common/
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace gps_serial {

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
    QSerialPort *m_serialGPS;

	void closeEvent(QCloseEvent *event); // Overloaded function
    bool open_serialGPS();
    bool m_flagSerialGPS = false;
    void calDDMM_MM(std::string data, double &latitude, double &longitude, int &status);

public Q_SLOTS:
    void GPS_CallBack();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
    bool modeFlag = true;

};

}  // namespace gps_serial

#endif // gps_serial_MAIN_WINDOW_H
