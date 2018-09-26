cmake_minimum_required(VERSION 2.8.3)
project(robothand_moveit_config)
find_package(catkin REQUIRED COMPONENTS robothand_ikfast_hand_plugin)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
