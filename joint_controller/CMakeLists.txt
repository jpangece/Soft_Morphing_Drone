cmake_minimum_required(VERSION 3.0.2)
project(joint_controller)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamixel_workbench_toolbox
  message_generation
)

add_message_files(
  FILES
  MotorState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy dynamixel_workbench_toolbox message_runtime std_msgs
  LIBRARIES dynamixel_sdk
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ./include
  ../include
  ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")


#add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES})


#############
## Testing ##
#############

add_executable(joint_position_control src/joint_position_control.cpp)  
target_link_libraries(joint_position_control ${catkin_LIBRARIES})

add_executable(publish_joint_angle src/publish_joint_angle.cpp)  
target_link_libraries(publish_joint_angle ${catkin_LIBRARIES})

add_executable(main_node src/main_node.cpp)
target_link_libraries(main_node ${catkin_LIBRARIES})

add_executable(dxl_latency_test_node src/dxl_latency_test_node.cpp)
target_link_libraries(dxl_latency_test_node ${catkin_LIBRARIES})