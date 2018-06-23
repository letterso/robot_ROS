#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryGoal.h>
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
    judge=false;
    //as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this, _1));
    as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB, this));
    as_.start();
    ROS_INFO("action start");
 }
   
 ~FollowJointTrajectoryAction(void)
  {
  }

  //translate radian to data which control servo
 	int data_process(double radian)
 	{
  	 	int sentdata;
   		sentdata=(int)(radian/M_PI*180.0/300.0*1024.0);//sent to servo
   		return sentdata;
 	}

  //get private data 
  int* getdata()
  {
     return pos_int;
  }

  public:bool judge;
	bool finish()
	{
	 	return judge;
	}	

	void finish_n()
	{
    judge=false;
  }

  public: int sen_length;
  int get_length()
  {
     return sen_length;
  }

 void goalCB()
 {
    ROS_INFO("goal is receive"); 
    int i,j,Pos_length;
    judge=false;
    if(as_.isNewGoalAvailable())
     {
      points_=&(as_.acceptNewGoal()->trajectory.points);
      as_.setSucceeded();
      Pos_length=points_->size();     
      ROS_INFO("Pos_length:%d",Pos_length);  
      sen_length=Pos_length*6;
     }
    else
     {
       ROS_INFO("goal is not available"); 
     }
  
   for(i=0;i<Pos_length;i++)
   {
     for(j=0;j<6;j++)
      {
       pos[i*6+j]= points_->at(i).positions[j];
       angle=(int)(pos[i*6+j]/M_PI*180);
       pos_int[i*6+j]=data_process(pos[i*6+j]);
       ROS_INFO("position%d.%d=%d angle:%d",i,j,pos_int[i*6+j],angle);  
       //pos_int[i*3+j]=data_process(pos[i*3+j]);
      }
    ROS_INFO(" ");
   }
    //as_.setSucceeded();
   judge=true;
 } 

 void preemptCB()
 {
   ROS_INFO("%s: Preempted", action_name_.c_str());
   // set the action state to preempted
   as_.setPreempted();
 }

  protected: 
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  
  std::string action_name_;
  std::string frame;
  int angle;
   
  //to the client
  control_msgs::FollowJointTrajectoryResult result_;
  control_msgs::FollowJointTrajectoryFeedback feedback_;

  //receive
  control_msgs::FollowJointTrajectoryGoal goal_;
  const std::vector<trajectory_msgs::JointTrajectoryPoint> *points_;
  double pos[];
  
  //Publisher
  int pos_int[];
};

int robotcontroldata;
void robotcontrol_Callback(const std_msgs::Int16::ConstPtr& data)
{
  robotcontroldata = data->data;
}

int main(int argc, char** argv)
 {
   ros::init(argc, argv, "robothand_hard_driver");
   FollowJointTrajectoryAction followJointTrajectoryAction("six_hand_controller/follow_joint_trajectory");
   //serial
   /* ros::NodeHandle n_("~");
   std::string serial_port;
   n_.getParam("port", serial_port);
   ROS_INFO("port:%s",serial_port.c_str());
   serial::Serial ser;
   try
    {
      //ser.setPort("/dev/ttyACM1");
      ser.setPort(serial_port);
      ser.setBaudrate(57600);
      serial::Timeout To=serial::Timeout::simpleTimeout(1000);
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

*/
//pubish position message
ros::NodeHandle nh_;
//ros::Rate loop_rate(10);
//ros::AsyncSpinner spinner(2);
ros::Publisher Pub_n=nh_.advertise<std_msgs::Int16MultiArray>("robot_hand_control",72);
ros::Subscriber robotcontrol_sub = nh_.subscribe<std_msgs::Int16>("/robot/control", 10, robotcontrol_Callback);
std_msgs::Int16MultiArray pos_sent_n;
//spinner.start();
//ros::spin();
int pointdata[1000];
int * pos_int_int;
int count,length_n,length_n_all=0;

//数据压缩
int Max_data_length=36;
int pos_int_adjust[Max_data_length];
int length_quotient;//商
int length_remainder;//余数
int i,j;

//pos_sent_n.layout.dim.push_back(std_msgs::MultiArrayDimension());
//pos_sent_n.layout.dim[0].label="pos_data";
//pos_sent_n.layout.dim[0].stride=1;

 while(ros::ok())
  {
    if(followJointTrajectoryAction.finish())
    {
      pos_int_int=followJointTrajectoryAction.getdata();
      length_n=followJointTrajectoryAction.get_length();
      //pos_sent_n.data.reserve(length_n);
      //if (robotcontroldata == 1)
      //{
        memcpy(&pointdata[length_n_all],pos_int_int,sizeof(int)*length_n);
        length_n_all = length_n_all+length_n;
        ROS_INFO("length_n_all:%d %d",length_n_all,length_n);
      //}
      followJointTrajectoryAction.finish_n(); 
      //robotcontroldata = 0;
    }

    if (robotcontroldata == 2)
    {
      ROS_INFO("robotcontrolType:%d",robotcontroldata);
      //pos_int_int = &pointdata[0];
      length_n = length_n_all;
      length_n_all = 0;
      pos_sent_n.data.clear();
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
          pos_int_adjust[i*6]=pointdata[i*6*length_quotient+j*6];
		      pos_int_adjust[i*6+1]=pointdata[(i*6*length_quotient+1)+j*6];
		      pos_int_adjust[i*6+2]=pointdata[(i*6*length_quotient+2)+j*6];
          pos_int_adjust[i*6+3]=pointdata[(i*6*length_quotient+3)+j*6];
		      pos_int_adjust[i*6+4]=pointdata[(i*6*length_quotient+4)+j*6];
		      pos_int_adjust[i*6+5]=pointdata[(i*6*length_quotient+5)+j*6];
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
      //pos_sent_n.layout.dim[0].size=length_n;
      //data<36
      if((length_n<Max_data_length)||(length_n==Max_data_length))
      {
        for(count=0;count<length_n;count++)
        {
          pos_sent_n.data.push_back(pos_int_int[count]);
         // ROS_INFO("%d",pos_int_int[count]);
        }
      }
      length_n=0; 
      //Pub_n.publish(pos_sent_n);
      ROS_INFO("publish success");
      robotcontroldata = 0;
    }  
    //if(getchar()) break;

    //loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;
 }
