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
  ui.outputButton->setEnabled(false);
  ui.moveButton->setEnabled(false);
  ui.resetButton->setEnabled(false);
  ui.xAxisEdit->setEnabled(false);
  ui.yAxisEdit->setEnabled(false);
  ui.zAxisEdit->setEnabled(false);
  ui.rollEdit->setEnabled(false);
  ui.pitchEdit->setEnabled(false);
  ui.yawEdit->setEnabled(false);
  pathJudge = false;

  ReadSettings();
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
  QObject::connect(&qnode, SIGNAL(stateState()), this, SLOT(stateChange()));

  /*********************
    ** Auto Start
    **********************/
  if ( ui.checkbox_remember_settings->isChecked() ) {
    on_button_connect_clicked(true);
  }
}

MainWindow::~MainWindow() {}

//四元数转换到欧拉角,2:roll,x,1:pitch,y,0:yaw.z
Eigen::Vector3d MainWindow::Quaterniond2Euler(const double x, const double y,
                                              const double z, const double w) {
  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  return euler;
}

//欧拉角转换到四元数
Eigen::Quaterniond MainWindow::euler2Quaternion(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}


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
      ui.outputButton->setEnabled(true);
      ui.moveButton->setEnabled(true);
      ui.resetButton->setEnabled(true);
      ui.xAxisEdit->setEnabled(true);
      ui.yAxisEdit->setEnabled(true);
      ui.zAxisEdit->setEnabled(true);
      ui.rollEdit->setEnabled(true);
      ui.pitchEdit->setEnabled(true);
      ui.yawEdit->setEnabled(true);

      //save init pos
      initPos = qnode.stateData;
      /*ui.xAxisEdit->setText(QString("%1").arg(QString::number(initPos[0]*100, 'f', 2)));
      ui.yAxisEdit->setText(QString("%1").arg(QString::number(initPos[1]*100, 'f', 2)));
      ui.zAxisEdit->setText(QString("%1").arg(QString::number(initPos[2]*100, 'f', 2)));
      ui.rollEdit->setText(QString("%1").arg(QString::number(initPos[9], 'f', 2)));
      ui.pitchEdit->setText(QString("%1").arg(QString::number(initPos[8], 'f', 2)));
      ui.yawEdit->setText(QString("%1").arg(QString::number(initPos[7], 'f', 2)));*/
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

void MainWindow::stateChange()
{
  std::vector<float> stateGetData;
  stateGetData = qnode.stateData;
  if(stateGetData.size() >= 10)
  {
    ui.xAxisLabel->setText(QString("%1 mm").arg(QString::number(stateGetData[0]*100, 'f', 2)));
    ui.yAxisLabel->setText(QString("%1 mm").arg(QString::number(stateGetData[1]*100, 'f', 2)));
    ui.zAxisLabel->setText(QString("%1 mm").arg(QString::number(stateGetData[2]*100, 'f', 2)));
    ui.wQLabel->setText(QString("%1").arg(QString::number(stateGetData[3], 'f', 2)));
    ui.xQLabel->setText(QString("%1").arg(QString::number(stateGetData[4], 'f', 2)));
    ui.yQLabel->setText(QString("%1").arg(QString::number(stateGetData[5], 'f', 2)));
    ui.zQLabel->setText(QString("%1").arg(QString::number(stateGetData[6], 'f', 2)));
    ui.rollLabel->setText(QString("%1 rad").arg(QString::number(stateGetData[7], 'f', 2)));
    ui.pitchLabel->setText(QString("%1 rad").arg(QString::number(stateGetData[8], 'f', 2)));
    ui.yawLabel->setText(QString("%1 rad").arg(QString::number(stateGetData[9], 'f', 2)));
  }
}

//move real robot
void MainWindow::on_moveButton_clicked()
{
  if(!pathJudge)
  {
    std::vector<float> sendData;
    if(!ui.xAxisEdit->text().isEmpty()&
       !ui.yAxisEdit->text().isEmpty()&
       !ui.zAxisEdit->text().isEmpty()&
       !ui.rollEdit->text().isEmpty()&
       !ui.pitchEdit->text().isEmpty()&
       !ui.yawEdit->text().isEmpty())
    {
      sendData.push_back(1);
      sendData.push_back(ui.xAxisEdit->text().toFloat()/100.0f);
      sendData.push_back(ui.yAxisEdit->text().toFloat()/100.0f);
      sendData.push_back(ui.zAxisEdit->text().toFloat()/100.0f);
      sendData.push_back(ui.rollEdit->text().toFloat());
      sendData.push_back(ui.pitchEdit->text().toFloat());
      sendData.push_back(ui.yawEdit->text().toFloat());
      qnode.sendData(sendData);
    }
  }
  else
  {
    qnode.sendPath(getPathData,1);
  }
}

//move vitual robot
void MainWindow::on_outputButton_clicked()
{
  if(!pathJudge)
  {
    std::vector<float> sendData;
    if(!ui.xAxisEdit->text().isEmpty()&
       !ui.yAxisEdit->text().isEmpty()&
       !ui.zAxisEdit->text().isEmpty()&
       !ui.rollEdit->text().isEmpty()&
       !ui.pitchEdit->text().isEmpty()&
       !ui.yawEdit->text().isEmpty())
    {
      sendData.push_back(0);
      sendData.push_back(ui.xAxisEdit->text().toFloat()/100.0f);
      sendData.push_back(ui.yAxisEdit->text().toFloat()/100.0f);
      sendData.push_back(ui.zAxisEdit->text().toFloat()/100.0f);
      sendData.push_back(ui.rollEdit->text().toFloat());
      sendData.push_back(ui.pitchEdit->text().toFloat());
      sendData.push_back(ui.yawEdit->text().toFloat());
      qnode.sendData(sendData);
    }
  }
  else
  {
    qnode.sendPath(getPathData,0);
  }
}

void MainWindow::on_resetButton_clicked()
{
  pathJudge = false;
  getPathData.clear();
  //initPos.insert(initPos.begin(),1);
  //qnode.sendData(initPos);
}

void MainWindow::on_recordButton_clicked()
{
  pathJudge = true;
  std::vector<float> getData;
  if(!ui.xAxisEdit->text().isEmpty()&
     !ui.yAxisEdit->text().isEmpty()&
     !ui.zAxisEdit->text().isEmpty()&
     !ui.rollEdit->text().isEmpty()&
     !ui.pitchEdit->text().isEmpty()&
     !ui.yawEdit->text().isEmpty())
  {
    getData.push_back(ui.xAxisEdit->text().toFloat()/100.0f);
    getData.push_back(ui.yAxisEdit->text().toFloat()/100.0f);
    getData.push_back(ui.zAxisEdit->text().toFloat()/100.0f);
    Eigen::Quaterniond q = euler2Quaternion(ui.rollEdit->text().toFloat(),ui.pitchEdit->text().toFloat(),ui.yawEdit->text().toFloat());
    getData.push_back(q.w());
    getData.push_back(q.x());
    getData.push_back(q.y());
    getData.push_back(q.z());
    getPathData.push_back(getData);
  }
}

}  // namespace guinode
