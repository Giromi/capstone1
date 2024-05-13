#!/usr/bin/env python

import rospy

# Messages
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import matplotlib.pyplot as plt

# Module imports
from motion import MotionPlanner
from detector import LineDetector
from controller import Controller
# import getch
# import threading

SIM_RATE = 30

class Follower:
    def __init__(self):
        self.detector = LineDetector()
        self.motion_planner = MotionPlanner()
        self.controller = Controller(Kp=0.1, Ki=0.1, Kd=0.1, T=SIM_RATE)

        rospy.init_node('line_follower')
        self.rate = rospy.Rate(SIM_RATE) #30hz
        self.subscriber = rospy.Subscriber('camera/image', Image, self.camera_callback)
        self.publisher = rospy.Publisher('error', Float32, queue_size=10)

        self.is_running = False

    def run(self):
        while not rospy.is_shutdown():# and self.is_running:
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass
                # self.write_video()


    def write_video(self):
        for i in range(len(self.detector.video_data)):
            self.detector.video_writer.write(self.detector.video_data[i])

    def camera_callback(self, msg):
        # if not self.is_running:
        #     return
        # direction = self.detector.get_direction(message=msg, line_color='yellow', tol=15)
        # self.motion_planner.move(direction)

        direction, error = self.detector.get_direction_with_pid(message=msg, line_color='yellow', tol=15)
        control_input = self.controller.PID_control(error)
        self.motion_planner.move_control(direction, control_input)

        self.publisher.publish(error)

    def toggle_robot(self):
        while True:
            key_input = getch.getch()  # Get a single character input
            if key_input.lower() == 's':
                self.is_running = not self.is_running
                if self.is_running:
                    print("Robot started.")
                else:
                    print("Robot stopped.")

if __name__ == '__main__':
    follower = Follower()
    # thread = threading.Thread(target=follower.toggle_robot)
    # thread.start()
    follower.run()
