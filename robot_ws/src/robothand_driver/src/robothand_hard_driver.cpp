#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <control_msgs/FollowJointTrajectoryActionFeedback.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryFeedback.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <vector>

using namespace std;

class FollowJointTrajectoryAction
{
public:

 FollowJointTrajectoryAction(std::string name) :
   as_(nh_, name, false),
   action_name_(name)
 {
    as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB, this));
    as_.start();
    Pub_n = nh_.advertise<std_msgs::Int16MultiArray>("robot_hand_control",72);
    Sub_n = nh_.subscribe<std_msgs::Bool>("/moveit/judge", 1, boost::bind(&FollowJointTrajectoryAction::Sub_n_callback,this,_1));
    moveitJudge = false;
    feedback_.joint_names.resize(6);
    feedback_.actual.positions.resize(6);
    feedback_.desired.positions.resize(6);
    feedback_.error.positions.resize(6);
    feedback_.joint_names[0] = "Joint_1";
    feedback_.joint_names[1] = "Joint_2";
    feedback_.joint_names[2] = "Joint_3";
    feedback_.joint_names[3] = "Joint_4";
    feedback_.joint_names[4] = "Joint_5";
    feedback_.joint_names[5] = "Joint_6";
    ROS_INFO("action start");
 }
   
 ~FollowJointTrajectoryAction(void)
  {
  }

void Sub_n_callback(const std_msgs::Bool::ConstPtr& data)
{
  moveitJudge = data->data;
}

  //end data send
 void end_data_send(int * indata)
  {
     int *pos_end_data;
     pos_end_data = indata;
     if (moveitJudge)
     {
       for(int count=0;count<6;count++)
        {
          pos_sent_end.data.push_back(pos_end_data[count]);
          ROS_INFO("%d",pos_end_data[count]);
        }
       Pub_n.publish(pos_sent_end);
       pos_sent_end.data.clear();
       moveitJudge = false;
       ROS_INFO("publish success");
     }
     else
     {
       for(int count=0;count<6;count++)
        {
          pos_sent_end.data.push_back(pos_end_data[count]);
          ROS_INFO("%d",pos_end_data[count]);
        }
     }
  }
  //data send
  void data_send(int * indata,int sen_length)
  {
    //数据压缩
    int length_n = sen_length;
    int *pos_int_int;
    int Max_data_length=36;
    int pos_int_adjust[Max_data_length];
    int length_quotient;//商
    int length_remainder;//余数
    int i,j;
    pos_int_int = indata;
    std_msgs::Int16MultiArray pos_sent_n;
    //data>36
    if(length_n>Max_data_length)
    {
      length_quotient=(int)(length_n/Max_data_length);//商
      length_remainder=(int)((length_n%Max_data_length)/6);//余数
      if((length_quotient%2)==0)
      {
        length_remainder=length_remainder+1;
      }
      j=1;
	    for(i=0; i<(Max_data_length/6);i++)
      {
        pos_int_adjust[i*6]=pos_int_int[i*6*length_quotient+j*6];
		    pos_int_adjust[i*6+1]=pos_int_int[(i*6*length_quotient+1)+j*6];
		    pos_int_adjust[i*6+2]=pos_int_int[(i*6*length_quotient+2)+j*6];
        pos_int_adjust[i*6+3]=pos_int_int[(i*6*length_quotient+3)+j*6];
		    pos_int_adjust[i*6+4]=pos_int_int[(i*6*length_quotient+4)+j*6];
		    pos_int_adjust[i*6+5]=pos_int_int[(i*6*length_quotient+5)+j*6];
	      if(j<length_remainder)  
		    {
	        j=j+1;
		    }          		
		  }
      // pos_int_int=pos_int_adjust;
      for(i=0;i<Max_data_length;i++)
	    {
        pos_sent_n.data.push_back(pos_int_adjust[i]);
        ROS_INFO("adjust:%d",pos_int_adjust[i]);
	    }
	  }

    //data<36
    if((length_n<Max_data_length)||(length_n==Max_data_length))
    {
      for(int count=0;count<length_n;count++)
      {
        pos_sent_n.data.push_back(pos_int_int[count]);
        ROS_INFO("%d",pos_int_int[count]);
      }
    }
    length_n=0; 
    Pub_n.publish(pos_sent_n);
    ROS_INFO("publish success");
  }

  //translate radian to data which control servo
 	int data_process(double radian,int num)
 	{
  	 	int sentdata;
      if(num == 0)
      {
        sentdata=(int)(radian/M_PI*180.0/300.0*1024.0);
      }
        if(num >0 &&num<3)
      {
        sentdata=(int)(radian/M_PI*180.0/360.0*4096.0);
      }
        if(num >3 && num<6)
      {
        sentdata=(int)(radian/M_PI*180.0/220.0*1024.0);
      }
   		return sentdata;
 	}

 void goalCB()
 {
    ROS_INFO("goal is receive"); 
    int i,j,Pos_length;
    double points_end[6];
    int points_end_data[6];
    if(as_.isNewGoalAvailable())
     {
      points_=&(as_.acceptNewGoal()->trajectory.points);
      Pos_length=points_->size(); 
      for(int co =0; co<6;co++)
      {
        points_end[co] = points_->at(Pos_length-1).positions[co];// points_[Pos_length-6+co];
        feedback_.desired.positions[co] = points_->at(Pos_length-1).positions[co];
        feedback_.actual.positions[co] = points_->at(Pos_length-1).positions[co];
        feedback_.error.positions[co] = 0;
      }    
      ROS_INFO("Pos_length:%d",Pos_length);  
      //sen_length=Pos_length*6;
     }
    else
     {
       ROS_INFO("goal is not available"); 
     }

     for(j=0;j<6;j++)
      {
       points_end_data[j] = data_process(points_end[j],j);
       angle=(int)(points_end[j]/M_PI*180);
       ROS_INFO("position%d=%d angle:%d",j,points_end_data[j],angle);  
      }
      end_data_send(points_end_data);
   /*for(i=0;i<Pos_length;i++)
   {
     for(j=0;j<6;j++)
      {
       pos[i*6+j]= points_->at(i).positions[j];
       angle=(int)(pos[i*6+j]/M_PI*180);
       pos_int[i*6+j]=data_process(pos[i*6+j],);
       ROS_INFO("position%d.%d=%d angle:%d",i,j,pos_int[i*6+j],angle);  
       //pos_int[i*3+j]=data_process(pos[i*3+j]);
      }
    ROS_INFO(" ");
   }
   data_send(pos_int,Pos_length*6);*/
   as_.publishFeedback(feedback_);
   as_.setSucceeded();
 } 

 void preemptCB()
 {
   ROS_INFO("%s: Preempted", action_name_.c_str());
   // set the action state to preempted
   as_.setPreempted();
 }

  protected: 
  ros::NodeHandle nh_;
  ros::Publisher Pub_n;
  ros::Subscriber Sub_n;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  bool moveitJudge;
  std::string action_name_;
  std::string frame;
  std_msgs::Int16MultiArray pos_sent_end;
  int angle;
   
  //to the client
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  int sen_length;
  //receive
  control_msgs::FollowJointTrajectoryGoal goal_;
  const std::vector<trajectory_msgs::JointTrajectoryPoint> *points_;
  double pos[];
  
  //Publisher
  int pos_int[1000];
};

int main(int argc, char** argv)
 {
   ros::init(argc, argv, "robothand_hard_driver");
   FollowJointTrajectoryAction followJointTrajectoryAction("six_hand_controller/follow_joint_trajectory");
   //serial
   ros::NodeHandle n_("~");
   std::string serial_port;
   n_.getParam("port", serial_port);
   ROS_INFO("port:%s",serial_port.c_str());
   serial::Serial ser;
   try
    {
      //ser.setPort("/dev/ttyUSB1");
      ser.setPort(serial_port);
      ser.setBaudrate(57600);
      serial::Timeout To=serial::Timeout::simpleTimeout(10000);
      ser.setTimeout(To);
      ser.open();
    }
  catch(serial::IOException& e)
   {
      ROS_INFO_STREAM("serial port open unsuccessfully");
     // return -1;
   }
  if(ser.isOpen())
   {
      ROS_INFO_STREAM("serial port open successfully");
   }
  else
   { 
    // return -1;
   }

  ros::spin();
  return 0;
}
