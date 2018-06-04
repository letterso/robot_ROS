#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <boost/foreach.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <iostream>  
#include <sstream>
#include <stdio.h>
#include <math.h>

namespace enc=sensor_msgs::image_encodings;

//获取鼠标点击事件
int click_num=1;
cv::Point click_point[2];
image_transport::Publisher objectimage_pub; 

//彩色图回调参数
cv::Mat rgb_img=cv::Mat::zeros(480,640,CV_8UC3);
cv::Mat rgb_pub=cv::Mat::zeros(480,640,CV_8UC3);

//深度图回调参数
cv::Mat depth_img=cv::Mat::zeros(480,640,CV_32FC1);

//点云获取
pcl::PointCloud<pcl::PointXYZ> cloud_data;

//计算重心坐标
cv::Point3f count_point(cv::Point point1, cv::Point point2);
cv::Point3f count_centre;

//发布器定义
//ros::Publisher points_pub;
//ros::Publisher points_init_pub;
ros::Publisher ROI_pointcentre_pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_pass;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_sor;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_vox;
pcl::PointCloud<pcl::PointXYZ> cloud_data_final;
pcl::PointCloud<pcl::PointXYZ> cloud_data_process(pcl::PointCloud<pcl::PointXYZ> data)
{
  //Z轴的PassThrough滤波，范围0.1到0.7m
  cloud_data_pass=data.makeShared();
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_data_pass);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 0.7);
  pass.filter(cloud_data_final);

 //使用StatisticalOutlierRemoval滤波器移除离群点
 /* cloud_data_sor=cloud_data_final.makeShared();
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;  
  sor.setInputCloud(cloud_data_sor);  
  sor.setMeanK(60);  
  sor.setStddevMulThresh(1.0);  
  sor.filter(cloud_data_final);  
  
  //Voxel_grid滤波珊格处理0.015m^3
 /* cloud_data_vox=cloud_data_final.makeShared();
  pcl::VoxelGrid<pcl::PointXYZ> vox;  
  vox.setInputCloud(cloud_data_vox);   
  vox.setLeafSize(0.010,0.010,0.010);  
  vox.filter(cloud_data_final); */
  return cloud_data_final;
}

//点云获取

void Cloud_Points_Callback(const sensor_msgs::PointCloud2& Cloud_msg)    
{    
    pcl::PointCloud<pcl::PointXYZ> cloud;     
    pcl::fromROSMsg(Cloud_msg, cloud); 
    cloud_data= cloud; 
    // cloud_data= cloud_data_process(cloud); 
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_msg (new pcl::PointCloud<pcl::PointXYZ>);
   // cloud_data_msg=cloud_data.makeShared();
    //cloud_data_msg->header.frame_id = "camera_depth_frame";
     //points_pub.publish(cloud_data_msg); 
    //points_pub.publish(cloud_data);  
}  

//深度图回调函数
void image_Depth_Callback(const sensor_msgs::ImageConstPtr& Depth_msg)  
{  
  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat mask,img=cv::Mat::zeros(480,640,CV_32FC1);
  cv::Mat mask_0=cv::Mat::zeros(480,640,CV_32FC1);
  try
  {
    cv_ptr= cv_bridge::toCvShare(Depth_msg,enc::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s",e.what());
  }

  img=cv_ptr->image;
 // cv::imshow("Depth_Image", cv_ptr->image);
 // cv::waitKey(3); 
}  

//彩色图回调函数
//sensor_msgs::ImagePtr object_pub_msg;
void image_Rgb_Callback(const sensor_msgs::ImageConstPtr& Rgb_msg)  
{  
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr= cv_bridge::toCvShare(Rgb_msg,enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {count_centre;
    ROS_ERROR("cv_bridge exception: %s",e.what());
  }
  rgb_img=cv_ptr->image;
  //if((click_point[0].x!=0)&&(click_point[0].y!=0)&&(click_point[1].x!=0)&&(click_point[1].y!=0))
  //cv::rectangle(rgb_img,click_point[0],click_point[1],cv::Scalar(0,255,0),2);
  cv::imshow("Rgb_Image", rgb_img); 
  cv::waitKey(1); 
} 

void publish_imgtotf(cv::Mat img)
{
    sensor_msgs::ImagePtr msg_pub;
    msg_pub = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    objectimage_pub.publish(msg_pub);
}

//ROI区域回调函数
bool judge_pubtotf=false;
int count=0;
void ROI_Callback(const std_msgs::Int32MultiArray::ConstPtr& array)
{
  int i=0;
  int msgs[100];
  float x,y,z;
  float centre_data[4];
  std_msgs::Float32MultiArray pub_msgs;
  //sensor_msgs::ImagePtr msg_pub;
  cv::Point ROI_point[2];
  for(std::vector<int>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
	    {
		    msgs[i] = *it;
	    	i++;
     }
  if (i>4)
  {
    int class_num=(int)(i/5);
    for(i=0;i<class_num;i++)
      {
          ROI_point[0].x=msgs[1+i*5];
          ROI_point[0].y=msgs[2+i*5];
          ROI_point[1].x=msgs[3+i*5];
          ROI_point[1].y=msgs[4+i*5];
          count_centre=count_point(ROI_point[0],ROI_point[1]);
          x=count_centre.x*1000;y=count_centre.y*1000;z=count_centre.z*1000;
          //坐标修正
          //z = z * (1-(0.031*log(z)-0.04));
          if (centre_data[0] == 1) z = z +16.5;
          if (centre_data[0] == 2) z = z +10.0;
          centre_data[0]=msgs[0+i*5];centre_data[1]=x;centre_data[2]=y;centre_data[3]=z;
          ROS_INFO("class:%f,x=%fmm, y=%fmm, z=%fmm",centre_data[0],centre_data[1],centre_data[2],centre_data[3]);
          for(int count=0; count<4; count++)
             {
                pub_msgs.data.push_back(centre_data[count]);
             }
       }
          ROI_pointcentre_pub.publish(pub_msgs);
          pub_msgs.data.clear();
          judge_pubtotf=false;     
  }
  else  //if can't detect the object, publish image again
  {
    ROS_INFO("No object");
    judge_pubtotf=true;
  }
}

//计算重心坐标
cv::Point3f count_point(cv::Point point1, cv::Point point2)
{
  cv::Point start,end;
  if(point1.x>point2.x) start.x=point2.x;
  else start.x=point1.x;
  if(point1.y>point2.y) start.y=point2.y;
  else start.y=point1.y;
  if(point1.x<point2.x) end.x=point2.x;
  else end.x=point1.x;
  if(point1.y<point2.y) end.y=point2.y;
  else end.y=point1.y;

  pcl::PointCloud<pcl::PointXYZ> cloud_data_rect;
  //int rect_width =end.x-start.x;
  //int rect_height=end.y-start.y;
  //cloud_data_rect.resize(rect_width*rect_height);
  cloud_data_rect.width=end.x-start.x;
  cloud_data_rect.height=end.y-start.y;
  cloud_data_rect.resize(cloud_data_rect.width*cloud_data_rect.height);
  //ROS_INFO("x_s = %d, y_s = %d, width= %d,height=%d",start.x,start.y,cloud_data_rect.width,cloud_data_rect.height);
  
  for(int x=0;x<cloud_data_rect.width;x++)
     {
        for(int y=0;y<cloud_data_rect.height;y++)
          {
            cloud_data_rect(x,y)=cloud_data((x+start.x),(y+start.y));
          }
     }
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_init_msg (new pcl::PointCloud<pcl::PointXYZ>);
  //cloud_data_init_msg=cloud_data_rect.makeShared();
  //cloud_data_init_msg->header.frame_id = "camera_depth_frame";
  //points_init_pub.publish(cloud_data_init_msg); 

  cloud_data_rect= cloud_data_process(cloud_data_rect); 
  Eigen::Vector4f centre;
  pcl::compute3DCentroid(cloud_data_rect,centre);
  cv::Point3f point_centre;
  point_centre.x=centre[0];
  point_centre.y=centre[1];
  point_centre.z=centre[2];
  
  return point_centre;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_msg (new pcl::PointCloud<pcl::PointXYZ>);
  // cloud_data_msg=cloud_data_rect.makeShared();
  //cloud_data_msg->header.frame_id = "camera_depth_frame";
  //points_pub.publish(cloud_data_msg); 
}

//获取鼠标点击
void onMouse (int event, int x, int y, int flags ,void* param)
{
  switch(event)
  {
     case CV_EVENT_LBUTTONDOWN:
     {
         publish_imgtotf(rgb_img);
     }
     break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pcl_test");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  objectimage_pub=it.advertise("/image/rgb/object",1); 
 // ros::Subscriber image_Depth_sub = nh.subscribe("/camera/depth/image", 1, image_Depth_Callback); 
  ros::Subscriber image_Rgb_sub = nh.subscribe("/camera/rgb/image_raw", 10, image_Rgb_Callback); 
  ros::Subscriber depth_Points_sub = nh.subscribe("/camera/depth/points", 10, Cloud_Points_Callback);
  ros::Subscriber ROI_sub = nh.subscribe<std_msgs::Int32MultiArray>("/image/ROI", 1, ROI_Callback);
  ROI_pointcentre_pub=nh.advertise<std_msgs::Float32MultiArray>("/camera/ROI_pointcentre", 30);
  //points_pub=nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/points_filter", 1); 
  //points_init_pub=nh.advertise< pcl::PointCloud<pcl::PointXYZ> >("/camera/depth/points_init", 1); 
  cv::namedWindow("Rgb_Image");
  cv::setMouseCallback("Rgb_Image",onMouse,0);
  
 // ros::spin();
   while(1)
  {
    rgb_pub=rgb_img.clone();
    if(!rgb_pub.empty())
    {
     // cv::imshow("Pub_Image", rgb_pub);
      if(judge_pubtotf)
       {
         publish_imgtotf(rgb_pub);
         judge_pubtotf=false;
       }
    }
    ros::spinOnce();
  }
  return 0;
}