#include <Event.h>
#include <Timer.h>

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

//////////////机械爪控制///////////////
Servo myservo;
#define Init_pose 90
#define min_pose 40
#define releaseTime 10
int outPose = Init_pose;
int step = 5;
int countTime = 0;

//////////////定时函数///////////////
Timer t_c,t_r,t_d;
void catchServo();
void releaseServo();
void Data_send();
int dataEvent = t_d.every(4800,Data_send);
int catchEvent = t_c.every(500,catchServo);
int releaseEvent = t_r.every(500,releaseServo);

void catchServo()
{
  if(outPose > min_pose)
  {
   outPose = outPose - step;
    myservo.write(outPose); 
    t_c.update();
   }
  else  
  {
    t_c.stop(catchEvent);
    catch_pub.publish(&finish_msg);
  }
 }
 
 void releaseServo()
{
  countTime = countTime + 1;
  if (countTime < releaseTime) 
  {
    t_r.update();
    myservo.write(Init_pose);
  }
  else
  {
    t_r.stop(releaseEvent);
    release_pub.publish(&finish_msg);
    countTime = 0;
    //Serial.print("release finish");
    }  
}

///////////////////////////////////ROS订阅器/////////////////////////////////
int* Joint;
int MultiArray_length;
int count_length;
void messageCb( const std_msgs::Int16MultiArray& pos_msg)
{
   MultiArray_length=((pos_msg.data_length)/6);
   Joint[(pos_msg.data_length)];
   count_length=MultiArray_length;
   Joint=pos_msg.data;
  //start send angle 
  dataEvent = t_d.every(4800,Data_send);  
  t_d.update(); 
  // FlexiTimer2::start();
}

void catch_subCb( const std_msgs::Bool& catch_sub_msg)
{
  catchEvent = t_c.every(500,catchServo);
  t_c.update();
}


void release_subCb( const std_msgs::Bool& releas_sub_msg)
{
  releaseEvent = t_r.every(500,releaseServo);
  t_r.update();
}

ros::Subscriber<std_msgs::Int16MultiArray> sub("robot_hand_control", &messageCb );
ros::Subscriber<std_msgs::Bool> catch_sub("/hand/control/catch", &catch_subCb );
ros::Subscriber<std_msgs::Bool> release_sub("/hand/control/release", &release_subCb );

///////////////////////压力传感器控制/////////////////////////////////
#define fsrRightPin A1
#define fsrLeftPin A2
int fsrRightReading,fsrLeftReading;
int averagePower = 0;
const int colaCatch = 200;
const int milkCatch = 300;

///////////////////////总线舵机控制/////////////////////////////////
SCServo smServo;
SCServo scsServo;

///////////////////////////////舵机参数/////////////////////////////////////////
#define Max_Angle_Limit 1023
#define Min_Angle_Limit 0
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

//////////////////////////////总线舵机发送/////////////////////////////////////
//////////////定时中断服务函数///////////////
int t = 800; 
int count = 0;
int J1_SendData,J2_SendData,J3_SendData,J4_SendData,J5_SendData,J6_SendData;
void Data_send()                    
{   
   if(count<count_length)
   {
   scsServo.WritePos(1, J1_SendData-Joint[count*6+0], t);
   smServo.WritePos(2, J2_SendData-Joint[count*6+1], t);
   smServo.WritePos(3, J3_SendData+Joint[count*6+2], t);
   smServo.WritePos(4, J4_SendData-Joint[count*6+3], t);
   scsServo.WritePos(5, J5_SendData-Joint[count*6+4], t);
   scsServo.WritePos(6, J6_SendData-Joint[count*6+5], t);
   count++;
  // dataEvent = t_d.every(1600,Data_send);  
   //t_d.update(); 
   }
   else
   {
    count=0;
    MultiArray_length=0;
    t_d.stop(dataEvent);
  //  J1_SendData=J1_Angle_Init;
   // J2_SendData=J2_Angle_Init;
   // J3_SendData=J3_Angle_Init;
   // J4_SendData=J4_Angle_Init;
  //  J5_SendData=J5_Angle_Init;
  //  J6_SendData=J6_Angle_Init;
   }
}

///////////////////////主函数/////////////////////////////////
void setup() 
{  
  //Serial1.begin(9600);
///////////////////////ROS初始化/////////////////////////////////
  nh.initNode();
  nh.subscribe(sub); 
  nh.subscribe(catch_sub);
  nh.subscribe(release_sub);
  nh.advertise(catch_pub);
  nh.advertise(release_pub);
  finish_msg.data = true;
  
///////////////////////机械夹爪初始化/////////////////////////////////  
 myservo.attach(9);
 myservo.write(Init_pose); 
  
///////////////////////总线舵机初始化/////////////////////////////////  
  scsServo.End = 1;
  smServo.End = 0;
  Serial2.begin(1000000); 
  scsServo.pSerial = &Serial2;
  smServo.pSerial = &Serial2;
  delay(1000);
  
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
  J1_SendData=J1_Angle_Init;
  J2_SendData=J2_Angle_Init;
  J3_SendData=J3_Angle_Init;
  J4_SendData=J4_Angle_Init;
  J5_SendData=J5_Angle_Init;
  J6_SendData=J6_Angle_Init;
}

void loop()
{ 
  fsrRightReading = analogRead(fsrRightPin);
  fsrLeftReading = analogRead(fsrLeftPin);
  averagePower = (fsrRightReading + fsrLeftReading) / 2;
  if (averagePower > colaCatch)
  {
    t_c.stop(catchEvent);
    catch_pub.publish(&finish_msg);
  }
    if (averagePower > milkCatch)
  {
    t_c.stop(catchEvent);
    catch_pub.publish(&finish_msg);
  }
  nh.spinOnce();
}

