#!/usr/bin/env python

# ROS packages (python)
import rospy

from geometry_msgs.msg import Twist

class MotionPlanner:
    def __init__(self, linear_speed=0.2, angular_speed=0.15):
        self.velocity = Twist()
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def move_control(self, direction, control_input):
        self.velocity.linear.x = self.linear_speed
        self.velocity.angular.z = control_input 
        self.publisher.publish(self.velocity)
        rospy.loginfo(f'Lin. vel. = {self.velocity.linear.x} - Ang. vel. = {self.velocity.angular.z}')

    def move(self, dir):
        if dir == 0:
            self.velocity.linear.x = 0
            self.velocity.angular.z = 0
            rospy.loginfo('Lin. vel. = 0')

        if dir == 1:
            self.velocity.linear.x = 0.2
            self.velocity.angular.z = 0
            rospy.loginfo('Lin. vel. = 0.2')

        if dir == 2:
            self.velocity.linear.x = .075
            self.velocity.angular.z = .15
            rospy.loginfo('Lin. vel. = 0.1 - Ang. vel. = 0.15')

        if dir == 3:
            self.velocity.linear.x = .075
            self.velocity.angular.z = -.15
            rospy.loginfo('Lin. vel. = 0.1 - Ang. vel. = -0.15')

        self.publisher.publish(self.velocity)

    def stop(self):
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0

