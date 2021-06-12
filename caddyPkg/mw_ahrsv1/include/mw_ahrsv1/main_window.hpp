/**
 * @file /include/mw_ahrsv1/main_window.hpp
 *
 * @brief Qt based gui for mw_ahrsv1.
 *
 * @date November 2010
 **/
#ifndef mw_ahrsv1_MAIN_WINDOW_H
#define mw_ahrsv1_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
#include <cstring>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace mw_ahrsv1 {

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
        QSerialPort *serial;
        caddy_msg::imu_msg imu_msg;
        bool open_serial();

public Q_SLOTS:
        void mw_ahrsv1CallBack();
        void on_pushButton_set_clicked();


private:
	Ui::MainWindowDesign ui;
	QNode qnode;
        void init_mw_ahrsv1();

        // ASCII CODE
        const unsigned char A  = 0x61;
        const unsigned char N  = 0x6E;
        const unsigned char G  = 0x67;
        const unsigned char M  = 0x6D;
        const unsigned char CR = 0x0D;
        const unsigned char LF = 0x0A;
        const unsigned char Y  = 0x79;
        const unsigned char R  = 0x72;
        const unsigned char SPACE = 0x32;

        typedef struct
        {
            // Euler
            float roll;
            float pitch;
            float yaw;
            float roll_acc;
            float pitch_acc;
            float yaw_acc;

        }Euler;
        Euler euler;

        typedef struct
        {
            // mag
            float x;
            float y;
            float z;

        }Mag;
        Mag mag;

        float past_yaw;
        bool isSet;
};

}  // namespace mw_ahrsv1

#endif // mw_ahrsv1_MAIN_WINDOW_H
