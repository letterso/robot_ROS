#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
 {
   ros::init(argc, argv, "robothand_tf_broadcaster");
   ros::NodeHandle n;
    
   ros::Rate r(100);
    
   tf::TransformBroadcaster broadcaster;
  
   while(n.ok())
   {
	  //parent到children的转换
	  //children收到的坐标数据通过tf转换到parent的坐标上
      broadcaster.sendTransform(
      tf::StampedTransform(
      tf::Transform(tf::Quaternion(1, 0, 0, 0),   //姿态偏差四元数
                	tf::Vector3(0.2217, 0.05109, 0.03252)), //坐标偏差 单位m0.2217, 0.05109, 0.01802
					ros::Time::now(),
					"base_hand",               //parent 
					"base_camrea"));           //children  
      r.sleep();
    }
 }