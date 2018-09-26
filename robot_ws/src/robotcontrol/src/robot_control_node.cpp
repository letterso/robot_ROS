#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>

//#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

using namespace std;
using namespace Eigen;

/*全局变量*/
bool getClassJudge = false, getClassPoint = true;
int classType = 1;
geometry_msgs::Pose class_point;
geometry_msgs::PointStamped hand_point;
ros::Publisher catch_pub;
ros::Publisher release_pub;
ros::Publisher moveitJudge_pub;

//欧拉角转换到四元数
Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());
  Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
  return q;
}

//四元数转换到欧拉角
Eigen::Vector3d Quaterniond2Euler(const double x, const double y,
                                  const double z, const double w) {
  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
  return euler;
}

//旋转矩阵到四元数
Eigen::Quaterniond rotationMatrix2Quaterniond(Eigen::Matrix3d R) {
  Eigen::Quaterniond q = Eigen::Quaterniond(R);
  q.normalize();
  return q;
}

//四元数到旋转矩阵
Eigen::Matrix3d Quaternion2RotationMatrix(const double x, const double y,
                                          const double z, const double w) {
  Eigen::Quaterniond q;
  q.x() = x;
  q.y() = y;
  q.z() = z;
  q.w() = w;
  Eigen::Matrix3d R = q.normalized().toRotationMatrix();
  return R;
}

// 根据首末点位置　和　运动步长　生成　直线轨迹点　容器
/*void makeLine(std::vector<geometry_msgs::Pose> waypoints,const
Eigen::Vector3d& start, const Eigen::Vector3d& stop, double ds)
{
  const Eigen::Vector3d travel = stop - start;//总向量 首位点位置差 向量
  const int steps = std::floor(travel.norm() / ds);// 向量长度/步长　得到步数
  geometry_msgs::Pose target_pose;

  // Linear interpolation
  for (int i = 0; i < steps; ++i)
  {
    double ratio = static_cast<float>(i) / steps;
    Eigen::Vector3d position = start + ratio * travel;//起点＋　每一步步长向量
    Eigen::Affine3d tr;
    tr = Eigen::Translation3d(position);
    waypoints.push_back(tr);
  }
}*/

void stateCatch(moveit::planning_interface::MoveGroupInterface &group) {
  moveit::planning_interface::MoveItErrorCode success;
  geometry_msgs::Pose target_pose;
  bool ItSuccess;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.clear();

  //设置初始位置
  group.setStartState(*group.getCurrentState());

  //设置运动路径
  target_pose.orientation.w = 1.000000;
  target_pose.orientation.x = 0.000000;
  target_pose.orientation.y = 0.000000;
  target_pose.orientation.z = 0.000000;
  target_pose.position.x = 0.000789;
  target_pose.position.y = -0.089177;
  target_pose.position.z = 0.247533;
  waypoints.push_back(target_pose);

  target_pose.orientation.w = 1.000000;
  target_pose.orientation.x = 0.000000;
  target_pose.orientation.y = 0.000000;
  target_pose.orientation.z = 0.000000;
  target_pose.position.x = -0.020000;
  target_pose.position.y = -0.005000;
  target_pose.position.z = 0.350000;
  waypoints.push_back(target_pose);

  //进行运动规划
  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01, // eef_step,
                                               0.0,  // jump_threshold
                                               trajectory_msg, false);
  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
           fraction * 100.0);

  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(),
                                       "hand");
  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  ItSuccess = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s", ItSuccess ? "SUCCEDED" : "FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  //输出运动
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;
  sleep(1.0);
  group.execute(plan);
  sleep(1.0);
  // sleep(3);
  // catch_pub.publish(pub_msgs);
  // sleep(2);
}

//////////////////////////////抓取-放置////////////////////////////////////////////
//是否完成抓取
bool classCatchJudge = false;
void classCatch_sub_Callback(const std_msgs::Bool::ConstPtr &data) {
  classCatchJudge = data->data;
}

//完成物体抓取，放到车上
void stateRecuse(moveit::planning_interface::MoveGroupInterface &group) {
  moveit::planning_interface::MoveItErrorCode success;
  geometry_msgs::Pose target_pose;
  bool ItSuccess;
  std::vector<geometry_msgs::Pose> waypoints;
  waypoints.clear();

  //设置初始位置
  group.setStartState(*group.getCurrentState());

  //设置约束
  moveit_msgs::Constraints endEffector_constraints;
  moveit_msgs::OrientationConstraint ocm1;
  ocm1.link_name = "gripper";
  ocm1.link_name = "Link_6";
  ocm1.header.frame_id = "base_link";
  // ocm1.orientation.w = 1.0;//四元数约束
  ocm1.absolute_x_axis_tolerance = 0.01; //欧拉角约束
  ocm1.absolute_y_axis_tolerance = 2 * 3.14;
  ocm1.absolute_z_axis_tolerance = 0.01;
  ocm1.weight = 1.0; //此限制权重
  endEffector_constraints.orientation_constraints.push_back(ocm1);

  moveit_msgs::OrientationConstraint ocm2;
  ocm2.link_name = "Link_4";
  ocm2.link_name = "Link_5";
  ocm2.header.frame_id = "base_link";
  ocm2.absolute_x_axis_tolerance = 0.01; //欧拉角约束
  ocm2.absolute_y_axis_tolerance = 0.01;
  ocm2.absolute_z_axis_tolerance = 0.01;
  ocm2.weight = 2.0; //此限制权重
  endEffector_constraints.orientation_constraints.push_back(ocm2);
  group.setPathConstraints(endEffector_constraints);

  //设置运动路径
  target_pose.orientation.w = 1.000000;
  target_pose.orientation.x = 0.000000;
  target_pose.orientation.y = 0.000000;
  target_pose.orientation.z = 0.000000;
  target_pose.position.x = 0.000809;
  target_pose.position.y = -0.240150;
  target_pose.position.z = 0.307580;
  waypoints.push_back(target_pose);

  target_pose.orientation.w = 0.707080;
  target_pose.orientation.x = 0.000000;
  target_pose.orientation.y = 0.707080;
  target_pose.orientation.z = 0.000000;
  target_pose.position.x = 0.307586;
  target_pose.position.y = -0.240195;
  target_pose.position.z = -0.000819;
  waypoints.push_back(target_pose);

  target_pose.orientation.w = 0.000000;
  target_pose.orientation.x = 0.000000;
  target_pose.orientation.y = 1.000000;
  target_pose.orientation.z = 0.000000;
  target_pose.position.x = -0.000337;
  target_pose.position.y = -0.240185;
  target_pose.position.z = -0.307591;
  waypoints.push_back(target_pose);

  target_pose.orientation.w = 0.000000;
  target_pose.orientation.x = 0.000000;
  target_pose.orientation.y = 1.000000;
  target_pose.orientation.z = 0.000000;
  target_pose.position.x = -0.000741;
  target_pose.position.y = -0.104373;
  target_pose.position.z = -0.264287;
  waypoints.push_back(target_pose);

  //进行运动规划
  moveit_msgs::RobotTrajectory trajectory_msg;
  double fraction = group.computeCartesianPath(waypoints,
                                               0.01, // eef_step,
                                               0.0,  // jump_threshold
                                               trajectory_msg, false);
  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
           fraction * 100.0);
  // trajectory_msg.joint_trajectory;
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(),
                                       "hand");
  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), trajectory_msg);
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  ItSuccess = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s", ItSuccess ? "SUCCEDED" : "FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  //输出运动
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;
  sleep(1.0);
  group.execute(plan);
  sleep(1.0);

  //清除约束
  group.clearPathConstraints();
  // release_pub.publish(pub_msgs);
  //  sleep(2);
}

//////////////////////////////放置-初始////////////////////////////////////////////
//是否完成放开
bool classReleaseJudge = false;
void classRelease_sub_Callback(const std_msgs::Bool::ConstPtr &data) {
  classReleaseJudge = data->data;
}

//完成物体放置，恢复初始姿态
void stateInit(moveit::planning_interface::MoveGroupInterface &group) {
  moveit::planning_interface::MoveItErrorCode success;
  std_msgs::Bool pub_msgs;
  pub_msgs.data = true;
  //进行运动规划
  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("catch_put_2");
  moveit::planning_interface::MoveGroupInterface::Plan plan3_1;
  ROS_INFO("Visualizing plan 3_1 (stateInit pose) %s",
           success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS"
                                                             : "FAILED");
  if (success == moveit_msgs::MoveItErrorCodes::SUCCESS)
    group.execute(plan3_1);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("catch_put_1_5");
  moveit::planning_interface::MoveGroupInterface::Plan plan3_2;
  success = group.plan(plan3_2);
  ROS_INFO("Visualizing plan 3_2 (stateInit pose) %s",
           success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS"
                                                             : "FAILED");
  if (success == moveit_msgs::MoveItErrorCodes::SUCCESS)
    group.execute(plan3_2);

  group.setStartState(*group.getCurrentState());
  group.setNamedTarget("init_pose");
  moveit::planning_interface::MoveGroupInterface::Plan plan3_3;
  success = group.plan(plan3_3);
  ROS_INFO("Visualizing plan 3_3 (stateInit pose) %s",
           success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS"
                                                             : "FAILED");
  if (success == moveit_msgs::MoveItErrorCodes::SUCCESS) {
    moveitJudge_pub.publish(pub_msgs);
    group.execute(plan3_3);
  }
}

moveit_msgs::RobotTrajectory jointTrajectorySyn(
    moveit::planning_interface::MoveGroupInterface &group,
    const std::vector<trajectory_msgs::JointTrajectory> jointData) {
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header = jointData[0].header;
  result.joint_names = jointData[0].joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (int i = 0; i < jointData.size(); i++) {
    for (int j = 0; j < jointData[i].points.size(); j++)
     {
      trajectory_msgs::JointTrajectoryPoint pt;
      pt = jointData[i].points[j];
     // pt.time_from_start = ros::Duration(time_offset);

      result.points.push_back(pt);
    }
  }

  moveit_msgs::RobotTrajectory moveit_result;
  moveit_result.joint_trajectory = result;

  //优化轨迹运动参数
  // The trajectory needs to be modified so it will include velocities as well.
  // First to create a RobotTrajectory object
  robot_trajectory::RobotTrajectory rt(group.getCurrentState()->getRobotModel(),
                                       "hand");
  // Second get a RobotTrajectory from trajectory
  rt.setRobotTrajectoryMsg(*group.getCurrentState(), moveit_result);
  // Thrid create a IterativeParabolicTimeParameterization object
  trajectory_processing::IterativeParabolicTimeParameterization iptp;
  // Fourth compute computeTimeStamps
  bool ItSuccess = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s", ItSuccess ? "SUCCEDED" : "FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(moveit_result);

  //返回结果
  return moveit_result;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "robomoveit");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(0);

  moveit::planning_interface::MoveGroupInterface handGroup("hand");
  moveit::planning_interface::MoveGroupInterface groupGet("hand");
  moveit::planning_interface::MoveGroupInterface groupCatch("hand");
  moveit::planning_interface::MoveGroupInterface groupRelease("hand");

  // ros::Subscriber ROI_pointcentre_sub =
  // nh.subscribe<std_msgs::Float32MultiArray>("/camera/ROI_pointcentre", 1,
  // ROI_pointcentre_Callback);
  ros::Subscriber classCatch_sub = nh.subscribe<std_msgs::Bool>(
      "/hand/classCatch", 1, classCatch_sub_Callback);
  ros::Subscriber classRelease_sub = nh.subscribe<std_msgs::Bool>(
      "/hand/classRelease", 1, classRelease_sub_Callback);
  moveitJudge_pub = nh.advertise<std_msgs::Bool>("/moveit/judge", 10);
  catch_pub = nh.advertise<std_msgs::Bool>("/hand/control/catch", 10);
  release_pub = nh.advertise<std_msgs::Bool>("/hand/control/release", 10);
  spinner.start();

  // init position:x=0.000802,y=-0.093441,z=0.160843  单位m
  // init orientation:w=0.707102,z=-0.707102,y=0.002474,z=0.002476
  //获取当前姿态
  geometry_msgs::Pose current_pose;
  current_pose = handGroup.getCurrentPose().pose;
  ROS_INFO("current position:x=%f,y=%f,z=%f", current_pose.position.x,
           current_pose.position.y, current_pose.position.z);
  ROS_INFO("current orientation:w=%f,x=%f,y=%f,z=%f",
           current_pose.orientation.w, current_pose.orientation.x,
           current_pose.orientation.y, current_pose.orientation.z);

  getClassPoint = true;
  classCatchJudge = false;
  classReleaseJudge = false;
  while (1) {
    if (getClassPoint) //初始-抓取
    {
      stateCatch(groupGet);
      getClassPoint = false;
    }
    if (classCatchJudge) //抓取-放置
    {
      stateRecuse(groupCatch);
      classCatchJudge = false;
    }
    if (classReleaseJudge) //放置-初始
    {
      stateInit(groupRelease);
      classReleaseJudge = false;
    }
    if (getchar())
      break;
  }
  return 0;
}