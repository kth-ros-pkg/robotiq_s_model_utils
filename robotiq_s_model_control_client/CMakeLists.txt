cmake_minimum_required(VERSION 2.8.3)
project(robotiq_s_model_control_client)

find_package(catkin REQUIRED COMPONENTS roscpp robotiq_s_model_control)

catkin_python_setup()

catkin_package(
  DEPENDS 
  CATKIN_DEPENDS roscpp robotiq_s_model_control
  INCLUDE_DIRS include
  LIBRARIES robotiq_s_model_control_client
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

add_library(${PROJECT_NAME} src/RobotiqSModelControlClient.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(robotiq_control_client_example src/robotiq_control_client_example.cpp)
target_link_libraries(robotiq_control_client_example robotiq_s_model_control_client ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} robotiq_control_client_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/robotiq_control_client_example_python
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)