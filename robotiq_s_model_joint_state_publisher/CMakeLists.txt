cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_joint_state_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS roscpp sensor_msgs
  INCLUDE_DIRS
  LIBRARIES 
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_executable(robotiq_dummy_joint_state_publisher src/robotiq_dummy_joint_state_publisher.cpp)
target_link_libraries(robotiq_dummy_joint_state_publisher ${catkin_LIBRARIES})

install(TARGETS robotiq_dummy_joint_state_publisher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)