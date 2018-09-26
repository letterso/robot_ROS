/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/guinode/qnode.hpp"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace guinode {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
  init_argc(argc),
  init_argv(argv)
{}

QNode::~QNode() {
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  wait();
}

void QNode::state_subscribe_callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  stateData.clear();
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
  {
    stateData.push_back(*it);
  }
  Q_EMIT stateState();
}

bool QNode::init() {
  ros::init(init_argc,init_argv,"guinode");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.
  state_subscribe = n.subscribe<std_msgs::Float32MultiArray>("/robot/robotState", 10, boost::bind(&QNode::state_subscribe_callback,this,_1));
  path_publisher = n.advertise<geometry_msgs::PoseArray>("/robot/pathData", 100);
  joint_publisher = n.advertise<std_msgs::Float32MultiArray>("/robot/jointData", 100);
  start();
  return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
  std::map<std::string,std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings,"guinode");
  if ( ! ros::master::check() ) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.
  ros::NodeHandle n;

  // Add your ros communications here.  
  state_subscribe = n.subscribe<std_msgs::Float32MultiArray>("/robot/robotState", 10, boost::bind(&QNode::state_subscribe_callback,this,_1));
  path_publisher = n.advertise<geometry_msgs::PoseArray>("/robot/pathData", 100);
  joint_publisher = n.advertise<std_msgs::Float32MultiArray>("/robot/jointData", 100);
  start();
  return true;
}

void QNode::sendData(std::vector<float> getData)
{
  publishJointData.data.clear();
  for(int i = 0; i<getData.size();i++)
  {
    publishJointData.data.push_back(getData[i]);
  }
  joint_publisher.publish(publishJointData);
}

void QNode::sendPath(std::vector< std::vector<float> > getData, unsigned int seq)
{
  geometry_msgs::PoseArray publicPathData;
  geometry_msgs::Pose PathData;
  publicPathData.header.frame_id = std::to_string(seq);
  //publicPathData.header.seq = seq;
  for(int i = 0; i<getData.size();i++)
  {
    PathData.position.x = getData[i][0];
    PathData.position.y = getData[i][1];
    PathData.position.z = getData[i][2];
    PathData.orientation.w= getData[i][3];
    PathData.orientation.x= getData[i][4];
    PathData.orientation.y= getData[i][5];
    PathData.orientation.z= getData[i][6];
    publicPathData.poses.push_back(PathData);
  }
  path_publisher.publish(publicPathData);
}

void QNode::run()
{
  ros::Rate loop_rate(50);
  while ( ros::ok() )
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

}

