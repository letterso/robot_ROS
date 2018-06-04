#include <iostream>  
#include <Eigen/Eigen>  
#include <stdlib.h>  
#include <Eigen/Geometry>  
#include <Eigen/Core>  
#include <vector>  
#include <math.h> 
#include <sstream>
#include <stdio.h> 

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/robot_state.h>
#include "actionlib/client/simple_action_client.h"  
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace Eigen;

/*机器人初始姿态参考参数
初始旋转矩阵：
-1  0  0
 0  1  0
 0  0 -1

四元数：
0 0 1 0

欧拉角：
yaw:2*pi pitch:0 roll:2*pi

坐标：（cm）
16.085 0 7.999
*/

/*全局变量*/
bool getClassJudge = false, getClassPoint = true;
int classType = 1;
geometry_msgs::Pose class_point;
geometry_msgs::PointStamped hand_point;
ros::Publisher catch_pub;
ros::Publisher release_pub;
ros::Publisher moveitJudge_pub;

//欧拉角转换到四元数
Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw)  
{  
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());  
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());  
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());  
    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;  
    return q;  
}

//四元数转换到欧拉角
  Eigen::Vector3d Quaterniond2Euler(const double x,const double y,const double z,const double w)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;   
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler;  
}

//旋转矩阵到四元数
  Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R)  
{  
    Eigen::Quaterniond q = Eigen::Quaterniond(R); 
    q.normalize();  
    return q;  
}  

//四元数到旋转矩阵
Eigen::Matrix3d Quaternion2RotationMatrix(const double x,const double y,const double z,const double w)  
{  
    Eigen::Quaterniond q;  
    q.x() = x;  
    q.y() = y;  
    q.z() = z;  
    q.w() = w;    
    Eigen::Matrix3d R = q.normalized().toRotationMatrix();  
    return R;  
}  

//group.setStartState(getCurrentRobotState(planning_scene_monitor_));
//////////////////////////////初始-抓取////////////////////////////////////////////
//接收数据点
void ROI_pointcentre_Callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
  int i=0,j=0;
  float msgs[100];
  for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	{
		 msgs[i] = *it;
	   i++;
	}
  if(i>3)
  {
    j=i/4;
    for (i = 0; i < j; i++)
    {
      if (msgs[i*4]==classType)
      {
        class_point.position.x = msgs[i*4+1];
        class_point.position.y = -msgs[i*4+2];
        class_point.position.z = msgs[i*4+3];
        getClassJudge = true;
      }    
    }
  }
}

//转换函数
void transformPoint(const tf::TransformListener& listener)
{
  geometry_msgs::PointStamped camera_point;
  camera_point.header.frame_id = "base_camrea";
  camera_point.header.stamp = ros::Time();
  camera_point.point.x = class_point.position.x;
  camera_point.point.y = class_point.position.y;
  camera_point.point.z = class_point.position.z;

  if (getClassJudge)
  {
    try{
        listener.transformPoint("base_hand", camera_point, hand_point);
    
        ROS_INFO("base_camrea: (%.2f, %.2f. %.2f) -----> base_hand: (%.2f, %.2f, %.2f) at time %.2f",
        camera_point.point.x, camera_point.point.y, camera_point.point.z,
        hand_point.point.x, hand_point.point.y, hand_point.point.z, hand_point.header.stamp.toSec());
        getClassPoint = true;
       }
    catch(tf::TransformException& ex)
       {
        ROS_ERROR("Received an exception trying to transform a point from \"base_camrea\" to \"base_hand\": %s", ex.what());
       }
       getClassJudge = false;
  }
}

void stateCatch(moveit::planning_interface::MoveGroupInterface &group)
{
   //设置初始位置
   moveit::planning_interface::MoveItErrorCode success;
   std_msgs::Bool pub_msgs;
   pub_msgs.data = true;
   //进行运动规划
   group.setStartState(*group.getCurrentState());
   group.setNamedTarget("ready_work_pose");
   moveit::planning_interface::MoveGroupInterface::Plan plan1_1;
   success = group.plan(plan1_1);
   ROS_INFO("Visualizing plan 1_1 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
   if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS) group.execute(plan1_1);
 
   //设置抓取目标点 
   geometry_msgs::Pose target_pose;
   target_pose.orientation.w = 1.000000;
   target_pose.orientation.x = 0.000000;
   target_pose.orientation.y = 0.000000;
   target_pose.orientation.z = 0.000000; 
   //target_pose.position.x = hand_point.point.x;
   //target_pose.position.y = hand_point.point.y;
   //target_pose.position.z = hand_point.point.z;
   //test
   target_pose.position.x = -0.020000;
   target_pose.position.y = -0.005000;
   target_pose.position.z = 0.350000;

   //进行运动规划
   group.setStartState(*group.getCurrentState());
   group.setPoseTarget(target_pose);
   moveit::planning_interface::MoveGroupInterface::Plan plan1_2;
   success = group.plan(plan1_2);
   ROS_INFO("Visualizing plan 1_2 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
   if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  
   {
     moveitJudge_pub.publish(pub_msgs);
     group.execute(plan1_2);
   }

   sleep(3);
   catch_pub.publish(pub_msgs);
   sleep(2);
}

//////////////////////////////抓取-放置////////////////////////////////////////////
//是否完成抓取
bool classCatchJudge = false;
void classCatch_sub_Callback(const std_msgs::Bool::ConstPtr& data)
{
  classCatchJudge = data->data;
}

//完成物体抓取，放到车上
void stateRecuse(moveit::planning_interface::MoveGroupInterface &group)
{
  moveit::planning_interface::MoveItErrorCode success;
  std_msgs::Bool pub_msgs;
  pub_msgs.data = true;
  //进行运动规划
  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("catch_put_1");
  moveit::planning_interface::MoveGroupInterface::Plan plan2_1;
  success = group.plan(plan2_1);
  ROS_INFO("Visualizing plan 2_1 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  group.execute(plan2_1);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("catch_put_1_5");
  moveit::planning_interface::MoveGroupInterface::Plan plan2_2;
  success = group.plan(plan2_2);
  ROS_INFO("Visualizing plan 2_2 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  group.execute(plan2_2);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("put");
  moveit::planning_interface::MoveGroupInterface::Plan plan2_3;
  success = group.plan(plan2_3);
  ROS_INFO("Visualizing plan 2_3 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  
  {
    moveitJudge_pub.publish(pub_msgs);
    group.execute(plan2_3);
  }
  
   sleep(3);
   release_pub.publish(pub_msgs);
   sleep(2);
}

//////////////////////////////放置-初始////////////////////////////////////////////
//是否完成放开
bool classReleaseJudge = false;
void classRelease_sub_Callback(const std_msgs::Bool::ConstPtr& data)
{
  classReleaseJudge = data->data;
}

//完成物体放置，恢复初始姿态
void stateInit(moveit::planning_interface::MoveGroupInterface &group)
{
  moveit::planning_interface::MoveItErrorCode success;
  std_msgs::Bool pub_msgs;
  pub_msgs.data = true;
  //进行运动规划
  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("catch_put_2");
  moveit::planning_interface::MoveGroupInterface::Plan plan3_1;
  ROS_INFO("Visualizing plan 3_1 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  group.execute(plan3_1);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("catch_put_1_5");
  moveit::planning_interface::MoveGroupInterface::Plan plan3_2;
  success = group.plan(plan3_2);
  ROS_INFO("Visualizing plan 3_2 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  group.execute(plan3_2);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("init_pose");
  moveit::planning_interface::MoveGroupInterface::Plan plan3_3;
  success = group.plan(plan3_3);
  ROS_INFO("Visualizing plan 3_3 (catch pose) %s",success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS" : "FAILED");
  if (success  == moveit_msgs::MoveItErrorCodes::SUCCESS)  
  {
    moveitJudge_pub.publish(pub_msgs);
    group.execute(plan3_3);
  }
}  

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robomoveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);
  
  moveit::planning_interface::MoveGroupInterface handGroup("hand");
  moveit::planning_interface::MoveGroupInterface groupGet("hand");
  moveit::planning_interface::MoveGroupInterface groupCatch("hand");
  moveit::planning_interface::MoveGroupInterface groupRelease("hand");

  ros::Subscriber ROI_pointcentre_sub = nh.subscribe<std_msgs::Float32MultiArray>("/camera/ROI_pointcentre", 1, ROI_pointcentre_Callback);
  ros::Subscriber classCatch_sub = nh.subscribe<std_msgs::Bool>("/hand/classCatch", 1, classCatch_sub_Callback);
  ros::Subscriber classRelease_sub = nh.subscribe<std_msgs::Bool>("/hand/classRelease", 1, classRelease_sub_Callback);
  moveitJudge_pub=nh.advertise<std_msgs::Bool>("/moveit/judge", 10);
  catch_pub=nh.advertise<std_msgs::Bool>("/hand/control/catch", 10);
  release_pub=nh.advertise<std_msgs::Bool>("/hand/control/release", 10);
  //actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("six_hand_controller/follow_joint_trajectory", true);  
  
  //tf::TransformListener listener(ros::Duration(10));
  //ros::Timer timer = nh.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
  spinner.start();
 
 //init position:x=0.000802,y=-0.093441,z=0.160843  单位m
 //init orientation:w=0.707102,z=-0.707102,y=0.002474,z=0.002476
 //获取当前姿态
  geometry_msgs::Pose current_pose;
  current_pose = handGroup.getCurrentPose().pose;
  ROS_INFO("current position:x=%f,y=%f,z=%f",current_pose.position.x,current_pose.position.y,current_pose.position.z);   
  ROS_INFO("current orientation:w=%f,x=%f,y=%f,z=%f",current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);   

 getClassPoint = true;
 classCatchJudge = true;
 classReleaseJudge = true;
 while (1)
 {
    if(getClassPoint)//初始-抓取
   {
    stateCatch(groupGet);
    getClassPoint = false; 
   }
   if(classCatchJudge)//抓取-放置
   {
     stateRecuse(groupCatch);
     classCatchJudge = false;
   }
   if(classReleaseJudge)//放置-初始
   {
     stateInit(groupRelease);
     classReleaseJudge = false;
   }
  if(getchar()) break;
 }
  return 0;
}