#include <iostream>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <serial/serial.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "robot_state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    // message declarations
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "Joint_1";
    joint_state.name[1] = "Joint_2";
    joint_state.name[2] = "Joint_3";
    joint_state.name[3] = "Joint_4";
    joint_state.name[4] = "Joint_5";
    joint_state.name[5] = "Joint_6";

    while (ros::ok()) 
    {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.position[0] = 0.0;
        joint_state.position[1] = 0.0;
        joint_state.position[3] = 0.0;
        joint_state.position[4] = 0.0;
        joint_state.position[5] = 0.0;
        joint_state.position[6] = 0.0;

        loop_rate.sleep();
    }

    return 0;
  }