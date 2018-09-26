#include <iostream>   
#include <stdlib.h>  
#include <vector>  
#include <sstream>
#include <stdio.h> 

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <geometry_msgs/PointStamped.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getState");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  moveit::planning_interface::MoveGroupInterface group("hand");
  geometry_msgs::Pose current_pose;
  spinner.start();

  while (1)
 {
     if(getchar() == 'g')
     {
        current_pose = group.getCurrentPose().pose;
        ROS_INFO("current position:x=%f,y=%f,z=%f",current_pose.position.x,current_pose.position.y,current_pose.position.z);   
        ROS_INFO("current orientation:w=%f,x=%f,y=%f,z=%f",current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);   
     }  
     if(getchar() == 'c') break;
  }
  
  ros::shutdown(); 
  return 0;
}