#!/usr/bin/env python

import rospy

# Messages
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from detection_msgs.msg import BoundingBox, BoundingBoxes

# Module imports
from planner import MotionPlanner
from detector import LineDetector
from controller import Controller
# import getch
# import threading

SIM_RATE = 30
# 'L' : Left / 'R' : Right / 'G' : Go
NAVI = ['R', 'L', 'L', 'R', 'L', 'R', 'G', 'R'] 
NAVI = ['R', 'L', 'L', 'R', 'L', 'L', 'R']
NAVI = ['R', 'L', 'L', 'R', 'R', 'R', 'L', 'R']

CAMERA_WIDTH = 680
CAMERA_HEIGHT = 480


class Follower:
    def __init__(self):
        self.detector = LineDetector(navi=NAVI)
        self.motion_planner = MotionPlanner(linear_speed=0.8)
        # self.controller = Controller(Kp=0.0001, Ki=0.00000, Kd=0.000, T=SIM_RATE)
        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=0.000, T=SIM_RATE)
        # self.controller = Controller(Kp=1.0, Ki=0.00000, Kd=0.000, T=SIM_RATE)

        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=10.0, T=SIM_RATE)
        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=0.1, T=SIM_RATE)
        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=0.00000001, T=SIM_RATE)

        self.controller = Controller(Kp=0.03, Ki=0.00002, Kd=0.1, T=SIM_RATE)

        rospy.init_node('line_follower')
        self.rate = rospy.Rate(SIM_RATE) #30hz
        # self.subscriber = rospy.Subscriber('camera/image', Image, self.camera_callback)

        self.yolo_subscriber = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.D435_callback)
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

        rotate_flag, error = self.detector.get_direction_with_pid(message=msg, line_color=['green', 'yellow'], tol=50)
        control_input = self.controller.PID_control(error)
        # control_input = self.controller.P_control(error)
        self.motion_planner.move_control(rotate_flag, control_input)

        self.publisher.publish(error)
    def D435_callback(self, msg):
        print(msg.bounding_boxes)
        if len(msg.bounding_boxes):
            print(msg.bounding_boxes[0].xmin,msg.bounding_boxes[0].xmax)
            print(640/2, (msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax)/2)


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
