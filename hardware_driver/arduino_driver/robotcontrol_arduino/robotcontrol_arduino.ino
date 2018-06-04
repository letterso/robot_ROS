#include <FlexiTimer2.h>

#include <SCServo.h>
#include <SCSProtocol.h>

#include <Servo.h>

#include <ros.h>
#include <ArduinoHardware.h>
#include <std_msgs/Bool.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>

///////////////////////////////////ROS初始化/////////////////////////////////
ros::NodeHandle  nh;

///////////////////////////////////ROS发布器/////////////////////////////////
std_msgs::Bool finish_msg;

ros::Publisher catch_pub("/hand/classCatch", &finish_msg);
ros::Publisher release_pub("/hand/classReleas", &finish_msg);

///////////////////////串口屏通讯/////////////////////////////////
void off()
{  
  for (int i = 0; i < 3; i++)
  {
    Serial3.write(0xff);
  }
}

void play(int num,int data)
{
   Serial3.print("n");
   Serial3.print(num);
   Serial3.print(".val=");
   Serial3.print(data);
  off();
 }

//////////////机械爪控制///////////////
Servo myservo;
#define Init_pose 50
#define min_pose 25
int outPose = Init_pose;
int step = 1;

///////////////////////压力传感器控制/////////////////////////////////
#define fsrRightPin A1
#define fsrLeftPin A2
int fsrRightReading,fsrLeftReading;
int averagePower = 0;
const int colaCatch = 350;
const int milkCatch = 350;

//////////////定时Callback函数///////////////
void catchServo()
{
  fsrRightReading = analogRead(fsrRightPin);
  fsrLeftReading = analogRead(fsrLeftPin);
  averagePower = (fsrRightReading + fsrLeftReading) / 2;
  if (averagePower > colaCatch)
  {
    catch_pub.publish(&finish_msg);
    FlexiTimer2::stop();
  }
  else
  {
      if(outPose > min_pose)
       {
         outPose = outPose - step;
         myservo.write(outPose); 
        }
      else  
      {
       catch_pub.publish(&finish_msg);
       FlexiTimer2::stop();
       }
    }
 }

void releaseServo()
{
  if(outPose < Init_pose)
  {
    outPose = outPose + step;
    myservo.write(outPose); 
   }
    else  
    {
      release_pub.publish(&finish_msg);
      outPose = Init_pose;
      FlexiTimer2::stop();
    }
 }
 
///////////////////////////////////ROS订阅器/////////////////////////////////
int* Joint;
int MultiArray_length;
int count_length;
bool messageCb_judge,release_subCb_judge;

void messageCb( const std_msgs::Int16MultiArray& pos_msg)
{
   MultiArray_length=((pos_msg.data_length)/6);
   Joint[(pos_msg.data_length)];
   count_length=MultiArray_length;
   Joint=pos_msg.data;
   
  //start send angle 
  FlexiTimer2::set(650, Data_send);
  FlexiTimer2::start();
}

void catch_subCb( const std_msgs::Bool& catch_sub_msg)
{
  if(catch_sub_msg.data)
  {
    FlexiTimer2::set(15, catchServo);
    FlexiTimer2::start();
  }
}

void release_subCb( const std_msgs::Bool& releas_sub_msg)
{
    if(releas_sub_msg.data)
  {
    FlexiTimer2::set(15, releaseServo);
    FlexiTimer2::start();
  }
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("robot_hand_control", &messageCb );
ros::Subscriber<std_msgs::Bool> catch_sub("/hand/control/catch", &catch_subCb );
ros::Subscriber<std_msgs::Bool> release_sub("/hand/control/release", &release_subCb );

///////////////////////总线舵机控制/////////////////////////////////
SCServo smServo;
SCServo scsServo;

///////////////////////////////舵机参数//////////////////////////////
#define J1_Angle_Init 200
#define J1_Min_Angle_Limit 10
#define J1_Max_Angle_Limit 1013
#define J2_Angle_Init 2222
#define J2_Min_Angle_Limit 910
#define J2_Max_Angle_Limit 3500
#define J3_Angle_Init 2237
#define J3_Min_Angle_Limit 1300
#define J3_Max_Angle_Limit 3700
#define J4_Angle_Init 1520
#define J4_Min_Angle_Limit 480
#define J4_Max_Angle_Limit 2540
#define J5_Angle_Init 945
#define J5_Min_Angle_Limit 10
#define J5_Max_Angle_Limit 1013
#define J6_Angle_Init 484
#define J6_Min_Angle_Limit 10
#define J6_Max_Angle_Limit 1013

//////////////////////////////总线舵机中断函数/////////////////////////
int t = 600; 
int count = 0;
void Data_send()                    
{   
   if(count<count_length)
   {
    messageCb_judge = true;
   }
  else
   {
    count=0;
    MultiArray_length=0;
    FlexiTimer2::stop();
   }
}

///////////////////////主函数/////////////////////////////////
void setup() 
{  
///////////////////////ROS初始化/////////////////////////////////
  nh.initNode();
  nh.subscribe(sub); 
  nh.subscribe(catch_sub);
  nh.subscribe(release_sub);
  nh.advertise(catch_pub);
  nh.advertise(release_pub);
  finish_msg.data = true;

///////////////////////串口屏////////////////////////////////
  Serial3.begin(9600); 
  int count;
  for(count=0;count<24;count++)
  {
    play(count,1);
  }
  
///////////////////////机械夹爪初始化/////////////////////////////////  
 myservo.attach(9);
 outPose = Init_pose;
 myservo.write(Init_pose); 

///////////////////////总线舵机初始化/////////////////////////////////  
  scsServo.End = 1;
  smServo.End = 0;
  Serial2.begin(1000000); 
  scsServo.pSerial = &Serial2;
  smServo.pSerial = &Serial2;
  delay(1000);
  messageCb_judge = false;
  
///////////////////////总线舵机角度初始化/////////////////////////////////  
  scsServo.EnableTorque(1, 1);
  smServo.EnableTorque(2, 1);
  smServo.EnableTorque(3, 1);
  smServo.EnableTorque(4, 1);
  scsServo.EnableTorque(5, 1);
  scsServo.EnableTorque(6, 1);
  scsServo.WritePos(1, J1_Angle_Init, 1000);
  smServo.WritePos(2, J2_Angle_Init, 1000);
  smServo.WritePos(3, J3_Angle_Init, 1000);
  smServo.WritePos(4, J4_Angle_Init, 1000);
  scsServo.WritePos(5, J5_Angle_Init, 1000);
  scsServo.WritePos(6, J6_Angle_Init, 1000);
}

void loop()
{ 
  if(messageCb_judge)
  {
    scsServo.WritePos(1,J1_Angle_Init- Joint[count*6+0], t);
    smServo.WritePos(2, J2_Angle_Init-Joint[count*6+1], t);
    smServo.WritePos(3, J3_Angle_Init+Joint[count*6+2], t);
    smServo.WritePos(4, J4_Angle_Init-Joint[count*6+3], t);
    scsServo.WritePos(5, J5_Angle_Init-Joint[count*6+4], t);
    scsServo.WritePos(6, J6_Angle_Init-Joint[count*6+5], t);
    play(1,Joint[count*6+0]);
    play(2,Joint[count*6+1]);
    play(3,Joint[count*6+2]);
    play(4,Joint[count*6+3]);
    play(5,Joint[count*6+4]);
    play(6,Joint[count*6+5]);
    count++;
    messageCb_judge = false;
   }
   nh.spinOnce();
}
