# robot_ROS
Design of Control System of Screening and Fetching Robots Based on Stereo Vision Guided  
Establish time: 2018.06.04  
  

# Environment:  
  Window System: Window10; tensoflow1.6.0-GPU; OpenCV3.1.0;   
  Linux System: Ubuntu16.04; tensoflow1.6.0-CPU ;ROS Kinetic ; OpenCV3.3.1;  

# IDE  
  RoboWare Studio  

# Hardware  
  Arduino mega2560  
  stm32F1  

# Introduction  
  version:  
  object detection: Tensorflow Object Detection API  
  Object location: PCL  
  
  robothand:  
  moveit,trac_ik  

  

# Tree  
  src  
    ├── CMakeLists.txt -> /opt/ros/indigo/share/catkin/cmake/toplevel.cmake  
    ├── cv_bridge  
    ├── image_geometry  
    ├── kinect  
    ├── object  
    ├── robotcontrol  
    ├── robothand  
    ├── robothand_config  
    ├── robothand_driver  
    ├── trac_ik_kinematics_plugin  
    └── trac_ik_lib
