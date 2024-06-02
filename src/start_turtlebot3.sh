#!/bin/bash

# ROS 환경 설정
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 터틀봇3 모델 설정
export TURTLEBOT3_MODEL=burger

# Gazebo에서 터틀봇3 실행
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
