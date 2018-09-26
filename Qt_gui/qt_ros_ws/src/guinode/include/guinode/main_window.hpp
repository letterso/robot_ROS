/**
 * @file /include/guinode/main_window.hpp
 *
 * @brief Qt based gui for guinode.
 *
 * @date November 2010
 **/
#ifndef guinode_MAIN_WINDOW_H
#define guinode_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace guinode {

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

  Eigen::Vector3d Quaterniond2Euler(const double x, const double y,const double z, const double w);
  Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw);

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
  //void on_actionAbout_triggered();
	void on_button_connect_clicked(bool check );
	void on_checkbox_use_environment_stateChanged(int state);
  void stateChange();


 Q_SIGNALS:
    void publishJointData(std::vector<float> jointData);

private Q_SLOTS:
    void on_moveButton_clicked();

    void on_resetButton_clicked();

    void on_outputButton_clicked();

    void on_recordButton_clicked();

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
  std::vector<float> initPos;
  std::vector< std::vector<float> > getPathData;
  bool pathJudge;
};

}  // namespace guinode

#endif // guinode_MAIN_WINDOW_H
