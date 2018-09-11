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
#include "../include/guinode/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace guinode {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
 // QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  //固定窗口大小
  this->setFixedSize( this->width (),this->height ());
  ui.moveButton->setEnabled(false);
  ui.resetButton->setEnabled(false);
  ui.xAxisEdit->setEnabled(false);
  ui.yAxisEdit->setEnabled(false);
  ui.zAxisEdit->setEnabled(false);
  ui.wQEdit->setEnabled(false);
  ui.xQEdit->setEnabled(false);
  ui.yQEdit->setEnabled(false);
  ui.zQEdit->setEnabled(false);

  ReadSettings();
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
    ** Auto Start
    **********************/
  if ( ui.checkbox_remember_settings->isChecked() ) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
      ui.moveButton->setEnabled(true);
      ui.resetButton->setEnabled(true);
      ui.xAxisEdit->setEnabled(true);
      ui.yAxisEdit->setEnabled(true);
      ui.zAxisEdit->setEnabled(true);
      ui.wQEdit->setEnabled(true);
      ui.xQEdit->setEnabled(true);
      ui.yQEdit->setEnabled(true);
      ui.zQEdit->setEnabled(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}


/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

//void MainWindow::on_actionAbout_triggered() {
//    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
//}

/*****************************************************************************
** setting save and load
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "guinode");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "guinode");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace guinode


void guinode::MainWindow::on_moveButton_clicked()
{
    std::vector<float> sendData;
    if(!ui.xAxisEdit->text().isEmpty()&
       !ui.yAxisEdit->text().isEmpty()&
       !ui.zAxisEdit->text().isEmpty()&
       !ui.wQEdit->text().isEmpty()&
       !ui.xQEdit->text().isEmpty()&
       !ui.yQEdit->text().isEmpty()&
       !ui.zQEdit->text().isEmpty())
    {
      sendData.push_back(ui.xAxisEdit->text().toFloat());
      sendData.push_back(ui.yAxisEdit->text().toFloat());
      sendData.push_back(ui.zAxisEdit->text().toFloat());
      sendData.push_back(ui.wQEdit->text().toFloat());
      sendData.push_back(ui.xQEdit->text().toFloat());
      sendData.push_back(ui.yQEdit->text().toFloat());
      sendData.push_back(ui.zQEdit->text().toFloat());
      qnode.sendData(sendData);
    }
}

void guinode::MainWindow::on_resetButton_clicked()
{

}
