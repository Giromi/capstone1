cmake_minimum_required(VERSION 2.8.3)
project(actor_line_follower)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  geometry_msgs
  nav_msgs
  cv_bridge
  rospy
  std_msgs
  sensor_msgs
  message_generation
  detection_msgs
)

find_package(gazebo REQUIRED)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    actor_command
  CATKIN_DEPENDS
    gazebo_ros
    gazebo_plugins
    geometry_msgs
    nav_msgs

)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(actor_command src/actor_command.cpp)
target_link_libraries(actor_command ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

# For tc_simple_world_plugin plugin
add_library(ActorCollisionsPlugin SHARED src/ActorCollisionsPlugin.cc)
target_link_libraries(ActorCollisionsPlugin ${GAZEBO_LIBRARIES})
