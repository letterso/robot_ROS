/**
 * @file /include/guinode/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef guinode_QNODE_HPP_
#define guinode_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32MultiArray.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace guinode {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();

Q_SIGNALS:
  void rosShutdown();

public Q_SLOTS:
  void sendData(std::vector<float> getData);

private:
  std_msgs::Float32MultiArray publishJointData;
	int init_argc;
	char** init_argv;
  ros::Publisher joint_publisher;
  ros::Publisher chatter_publisher;
  bool publishJudge;
};

}  // namespace guinode

#endif /* guinode_QNODE_HPP_ */
