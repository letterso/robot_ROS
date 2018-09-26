#include <ros/ros.h>
#include <serial/serial.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

//运动控制
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

//坐标系统
#include <tf/transform_broadcaster.h>

// define	sBUFFERSIZE	4800            //send buffer size
// 串口发送缓存长度
#define rBUFFERSIZE 100 // receive buffer size 串口接收缓存长度
// unsigned char s_buffer[sBUFFERSIZE];  //发送缓存
unsigned char r_buffer[rBUFFERSIZE]; //接收缓存

using namespace std;

/************************************
 * 串口数据发送格式
  * head head joint1 joint2 joint3 joint4 joint5 joint6 CRC
  * 0xff 0xff  float  float  float  float  float  float  u8
 * **********************************/
/**********************************************************
 * 串口接收数据格式
  * head head joint1 joint2 joint3 joint4 joint5 joint6 CRC
  * 0xaa 0xaa  float  float  float  float  float  float  u8
 * ********************************************************/

//联合体，用于浮点数与16进制的快速转换
typedef union {
  unsigned char cvalue[4];
  float fvalue;
} float_union;

serial::Serial ser;

//机械臂状态发布器
ros::Publisher robot_tate_pub;

/**********************************************************
 * send，数据接打包与发送
 * ********************************************************/
void data_send(vector<vector<float>> joint_data) {
  float_union joint_union;
  //数据打包
  int sendBuffSize = joint_data.size() * joint_data[0].size() * 4 + 3;
  unsigned char s_buffer[sendBuffSize]; //发送缓存
  unsigned char CRC_buffer = 0x01;
  memset(s_buffer, 0, sizeof(s_buffer));
  s_buffer[0] = 0xff;
  s_buffer[1] = 0xff;
  for (int i = 0; i < joint_data.size(); i++) {
    for (int j = 0; j < joint_data[i].size(); j++) {
      joint_union.fvalue = joint_data[i][j];
      s_buffer[2 + i * 4] = joint_union.cvalue[0];
      s_buffer[3 + i * 4] = joint_union.cvalue[1];
      s_buffer[4 + i * 4] = joint_union.cvalue[2];
      s_buffer[5 + i * 4] = joint_union.cvalue[3];
    }
    CRC_buffer = CRC_buffer * s_buffer[2 + i * 4] * s_buffer[3 + i * 4] *
                 s_buffer[4 + i * 4] * s_buffer[5 + i * 4];
  }
  //发送数据
  ser.write(s_buffer, sendBuffSize);
}

/**********************************************************
 * receive，数据接受与处理
 * ********************************************************/
bool data_receive(unsigned char *buffer) {
  bool receive_success = false;
  unsigned char CRC;
  // int i;
  if ((buffer[0] == 0xaa) && (buffer[1] == 0xaa)) {
    CRC = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5] ^ buffer[6] ^ buffer[7];
    if (CRC == buffer[8])
      receive_success = 1; //校验通过，数据包正确
    else
      receive_success = 0; //校验失败，丢弃数据包
  }
  return receive_success;
}

void data_pub(unsigned char *buffer) {
  float_union joint_union;
  sensor_msgs::JointState robot_state_data;

  int i;
  for (i = 0; i < 6; i++) {
    joint_union.cvalue[0] = buffer[2 + 4 * i];
    joint_union.cvalue[1] = buffer[3 + 4 * i];
    joint_union.cvalue[2] = buffer[4 + 4 * i];
    joint_union.cvalue[3] = buffer[5 + 4 * i];
    robot_state_data.position.push_back(joint_union.fvalue);
  }
  robot_tate_pub.publish(robot_state_data);
}

class FollowJointTrajectoryAction {
public:
  FollowJointTrajectoryAction(std::string name)
      : as_(nh_, name, false), action_name_(name) {
    as_.registerGoalCallback(
        boost::bind(&FollowJointTrajectoryAction::goalCB, this));
    // as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB,
    // this));
    as_.start();
    // nh_.advertise<std_msgs::Int16MultiArray>("robot_hand_control",72);
    // Sub_n = nh_.subscribe<std_msgs::Bool>("/moveit/judge", 1,
    // boost::bind(&FollowJointTrajectoryAction::Sub_n_callback,this,_1));
    ROS_INFO("action start");
  }

  ~FollowJointTrajectoryAction(void) {}

  // translate radian to data which control servo
  vector<vector<float>> data_process(vector< vector<float> > joint_data) {
    return joint_data;
  }

  void goalCB() {
    ROS_INFO("goal is receive");
    vector<vector<float>> joint_data;
    vector<float> joint_data_round;
    int i, j, Pos_length;
    double points_end[6];
    int points_end_data[6];
    if (as_.isNewGoalAvailable()) {
      points_ = &(as_.acceptNewGoal()->trajectory.points);
      Pos_length = points_->size();
      for (int i = 0; i < Pos_length; i++) {
        for (int j = 0; j < 6; j++) {
          joint_data_round.push_back(points_->at(i).positions[j]);
        }
        joint_data.push_back(joint_data_round);
        joint_data_round.clear();
      }
    } else {
      ROS_INFO("goal is not available");
    }

    //发送数据
    data_send(data_process(joint_data));
    as_.setSucceeded();
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher Pub_n;
  ros::Subscriber Sub_n;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  bool moveitJudge;
  std::string action_name_;
  std::string frame;
  std_msgs::Int16MultiArray pos_sent_end;
  int angle;

  // to the client
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  int sen_length;

  // receive
  control_msgs::FollowJointTrajectoryGoal goal_;
  const std::vector<trajectory_msgs::JointTrajectoryPoint> *points_;
  double pos[];

  // Publisher
  int pos_int[1000];
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "robothand_stm32_driver");
  ros::NodeHandle nh;

  //机器人姿态发布器
  robot_tate_pub = nh.advertise<sensor_msgs::JointState>("/move_group/robothand_controller_joint_states",10);

  //打开串口
  try {
    ser.setPort("/dev/ttyUSB0");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }

  if (ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }

  // 10hz频率执行
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    if (ser.available()) {
      size_t bytes_read;
      bytes_read = ser.read(r_buffer, ser.available());
      if (r_buffer[0] == 0xaa & r_buffer[1] == 0xaa) {
        if (data_receive(r_buffer)) {
          data_pub(r_buffer);
        }
      }
    }
    loop_rate.sleep();
  }
}
