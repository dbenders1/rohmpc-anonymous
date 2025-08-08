cmake_minimum_required(VERSION 3.0.2)
project(simple_sim)

add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-fstack-protector)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  nav_msgs
  tf_conversions
  tf2_ros
  roscpp
  roslib
)

find_package(Eigen3 REQUIRED)
find_package(Boost)

add_message_files(
  FILES
  DroneFalconOutput.msg
  DroneFalconStepControl.msg
  DroneHovergamesControl.msg
)

add_service_files(
  FILES
  GetPhysicsProperties.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp roslib message_runtime
  DEPENDS Boost
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

list(APPEND LIBRARY_SOURCES
  src/instrumentation_timer.cpp
  src/ros_timer_handler.cpp
)

list(APPEND LIBRARY_HEADERS
  include/instrumentation_timer.h
  include/ros_timer_handler.h
)

add_library(${PROJECT_NAME} ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})

add_executable(falcon_model_node src/falcon_model_node.cpp)
add_dependencies(falcon_model_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(falcon_model_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(hovergames_model_node src/hovergames_model_node.cpp)
add_dependencies(hovergames_model_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(hovergames_model_node ${PROJECT_NAME} ${catkin_LIBRARIES})
