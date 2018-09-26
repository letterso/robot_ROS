#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <iostream>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
//#include <moveit_visual_tools/moveit_visual_tools.h>
//#include <visualization_msgs/Marker.h>

using namespace std;
using namespace Eigen;

// moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
//订阅器，发布器
ros::Subscriber targetPoint_sub;
ros::Subscriber pathPoints_sub;
ros::Publisher robotState_pub;
ros::Publisher marker_pub;
ros::Publisher plan_positions_pub;
//全局变量
geometry_msgs::Pose target_pose;
std::vector<float> arrayData;
std::vector<geometry_msgs::Pose> waypoints;
bool outTypeJudge;

//四元数转换到欧拉角,2:roll,x,1:pitch,y,0:yaw.z
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

//欧拉角转换到四元数
Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw) {
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
  return q;
}

//获取目标位置
void targetPoint_Callback(const std_msgs::Float32MultiArray::ConstPtr &array) {
  //设置目标点
  for (std::vector<float>::const_iterator it = array->data.begin();
       it != array->data.end(); ++it) {
    arrayData.push_back(*it);
  }
  target_pose.position.x = arrayData[1];
  target_pose.position.y = arrayData[2];
  target_pose.position.z = arrayData[3];
  Eigen::Quaterniond q =
      euler2Quaternion(arrayData[4], arrayData[5], arrayData[6]);
  target_pose.orientation.w = q.w();
  target_pose.orientation.x = q.x();
  target_pose.orientation.y = q.y();
  target_pose.orientation.z = q.z();
  /*arget_pose.orientation.w = arrayData[4];
  target_pose.orientation.x = arrayData[5];
  target_pose.orientation.y = arrayData[6];
  target_pose.orientation.z = arrayData[7];*/
  ROS_INFO("x: %f; y: %f; z: %f; qw: %f; qx: %f; qy: %f; qz: %f",
           target_pose.position.x, target_pose.position.y,
           target_pose.position.z, target_pose.orientation.w,
           target_pose.orientation.x, target_pose.orientation.y,
           target_pose.orientation.z);
}

void pathPoints_Callback(const geometry_msgs::PoseArray::ConstPtr &array) {
  waypoints.clear();
  for (std::vector<geometry_msgs::Pose>::const_iterator it =
           array->poses.begin();
       it != array->poses.end(); ++it) {
    waypoints.push_back(*it);
  }
  if (array->header.frame_id == "1")
    outTypeJudge = true;
  else
    outTypeJudge = false;
}

//发布机器人状态
void pubRobotState(moveit::planning_interface::MoveGroupInterface &group) {
  //获取当前姿态
  geometry_msgs::Pose current_pose;
  current_pose = group.getCurrentPose().pose;
  std_msgs::Float32MultiArray robotState;
  robotState.data.push_back(current_pose.position.x);
  robotState.data.push_back(current_pose.position.y);
  robotState.data.push_back(current_pose.position.z);
  robotState.data.push_back(current_pose.orientation.w);
  robotState.data.push_back(current_pose.orientation.x);
  robotState.data.push_back(current_pose.orientation.y);
  robotState.data.push_back(current_pose.orientation.z);
  Eigen::Vector3d euler =
      Quaterniond2Euler(current_pose.orientation.x, current_pose.orientation.y,
                        current_pose.orientation.z, current_pose.orientation.w);
  robotState.data.push_back(euler[2]);
  robotState.data.push_back(euler[1]);
  robotState.data.push_back(euler[0]);
  robotState_pub.publish(robotState);
}

//发布规划运动结果
void pubMotionData(trajectory_msgs::JointTrajectory planData) {
  sensor_msgs::JointState fake_robot_state;
  fake_robot_state.header = planData.header;
  ros::Time init_time(0.0);
  // init_time = plan.trajectory_.joint_trajectory.header.stamp;
  for (int i = 0; i < planData.points.size(); i++) {
    fake_robot_state.header.stamp =
        init_time + planData.points[i].time_from_start;
    fake_robot_state.position = planData.points[i].positions;
    fake_robot_state.velocity = planData.points[i].velocities;
    fake_robot_state.effort = planData.points[i].accelerations;
    plan_positions_pub.publish(fake_robot_state);
    fake_robot_state.position.clear();
    fake_robot_state.velocity.clear();
    fake_robot_state.effort.clear();
    // plan_robot_state = plan.trajectory_.joint_trajectory.points[i];
    // plan_robot_pub.publish(plan_robot_state);
  }
}

//笛卡尔坐标系轨迹运动规划
void cartesianPath(moveit::planning_interface::MoveGroupInterface &group,
                   bool outType) {
  group.setStartState(*group.getCurrentState());
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
  bool ItSuccess = iptp.computeTimeStamps(rt);
  ROS_INFO("Computed time stamp %s", ItSuccess ? "SUCCEDED" : "FAILED");
  // Get RobotTrajectory_msg from RobotTrajectory
  rt.getRobotTrajectoryMsg(trajectory_msg);

  //输出运动
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory_ = trajectory_msg;
  pubMotionData(plan.trajectory_.joint_trajectory);
  if (outType) {
    group.execute(plan);
  }
  waypoints.clear();
}

//点动运动
void pointMove(moveit::planning_interface::MoveGroupInterface &group) {
  moveit::planning_interface::MoveItErrorCode success;
  // group.setStartStateToCurrentState();
  group.setStartState(*group.getCurrentState());
  // joint space goal
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
  robot_state::RobotStatePtr robot_state_ptr(
      group.getCurrentState()); // copy state
                                // robot_state_ptr->setToDefaultValues();
  const robot_state::JointModelGroup *joint_model_group =
      robot_model_ptr->getJointModelGroup(group.getName());
  bool found_ik =
      robot_state_ptr->setFromIK(joint_model_group, target_pose, 10, 0.5);
  if (found_ik) {
    std::vector<double> jointValues;
    robot_state_ptr->copyJointGroupPositions("hand", jointValues);
    group.setJointValueTarget(jointValues);
    // group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    success = group.plan(plan);
    ROS_INFO("Visualizing plan %s",
             success == moveit_msgs::MoveItErrorCodes::SUCCESS ? "SUCCESS"
                                                               : "FAILED");
    if (success == moveit_msgs::MoveItErrorCodes::SUCCESS & arrayData[0] == 0) {
      pubMotionData(plan.trajectory_.joint_trajectory);
    }
    if (success == moveit_msgs::MoveItErrorCodes::SUCCESS & arrayData[0] == 1) {
      group.execute(plan);
    }
  } else
    ROS_INFO("setFromIK: FAILED");
  arrayData.clear();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointMove");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);

  moveit::planning_interface::MoveGroupInterface group("hand");
  /*marker_pub =
      nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);*/
  targetPoint_sub = nh.subscribe<std_msgs::Float32MultiArray>(
      "/robot/jointData", 10, targetPoint_Callback);
  pathPoints_sub = nh.subscribe<geometry_msgs::PoseArray>("/robot/pathData", 10,
                                                          pathPoints_Callback);
  robotState_pub =
      nh.advertise<std_msgs::Float32MultiArray>("/robot/robotState", 10);
  /* visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(
       "base_link", "/moveit_visual_markers"));*/
  plan_positions_pub =
      nh.advertise<sensor_msgs::JointState>("/plan/fake_robot_state", 100);
  ros::Publisher plan_robot_pub =
      nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/plan/robot_state",
                                                          100);
  // trajectory_msgs::JointTrajectoryPoint plan_robot_state;

  spinner.start();
  ros::Rate loop_rate(10);
  while (ros::ok) {
    pubRobotState(group);
    if (arrayData.size() > 0) {
      pointMove(group);
    }
    if (waypoints.size() > 0) {
      cartesianPath(group, outTypeJudge);
    }
    loop_rate.sleep();
  }
  return 0;
}
