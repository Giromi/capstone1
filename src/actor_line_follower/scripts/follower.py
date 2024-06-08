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

# 'L' : Left / 'R' : Right / 'G' : Go


class Follower:
    def __init__(self, navi, sim_rate, camera_width, camera_height):
        self.detector = LineDetector()
        self.motion_planner = MotionPlanner(linear_speed=0.8)
        self.navi = navi
        self.sim_rate = sim_rate
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.ref = camera_height / 2
        # self.controller = Controller(Kp=0.0001, Ki=0.00000, Kd=0.000, T=SIM_RATE)
        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=0.000, T=SIM_RATE)
        # self.controller = Controller(Kp=1.0, Ki=0.00000, Kd=0.000, T=SIM_RATE)

        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=10.0, T=SIM_RATE)
        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=0.1, T=SIM_RATE)
        # self.controller = Controller(Kp=0.01, Ki=0.00000, Kd=0.00000001, T=SIM_RATE)

        self.controller = Controller(Kp=0.03, Ki=0.00002, Kd=0.1, T=SIM_RATE)

        rospy.init_node('line_follower')
        self.rate = rospy.Rate(self.sim_rate) #30hz
        self.subscriber = rospy.Subscriber('camera/image', Image, self.camera_callback)

        # self.yolo_subscriber = rospy.Subscriber('/yolov5/detections', BoundingBoxes, self.D435_callback)
        self.publisher = rospy.Publisher('error', Float32, queue_size=10)

        self.is_running = False
        self.navi = navi

        self.error = None
        self.error_stop_old = None
        self.error_go_old = None

        self.rotate_sign = None
        self.rotate_sign_old = None

        self.stop_height_old = None

        self.check_front = False

    def run(self):
        while not rospy.is_shutdown():# and self.is_running:
            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass
                # self.write_video()

    def examine_path(self, error_stop, error_go, stop_heigth):
        self.error = error_go if error_stop == None else error_stop
        if self.error == None:
            self.error = 0

        navi_size = len(self.navi)

        rotate_sign = None
        if self.rotate_sign == None and error_stop != None:
            self.rotate_sign = 0

        if error_stop == None and self.error_stop_old != None \
            and navi_size:
            rospy.logwarn('rotate flag')
            if self.navi[0] == 'L':
                self.rotate_sign = 1
            elif self.navi[0] == 'R':
                self.rotate_sign = -1

        if not self.check_front and error_go and self.error_go_old and abs(error_go) <= 100:
            if abs(error_go) > abs(self.error_go_old):
                self.check_front = False
            elif self.rotate_sign == 1 or self.rotate_sign == -1:
                rospy.logwarn('stop rotate')
                self.check_front = True
                self.error = 0
        elif self.check_front and self.rotate_sign and error_go and self.error_go_old: 
            if abs(error_go) <= abs(self.error_go_old) and 80 <= abs(error_go) <= 100:
                self.rotate_sign = None
                self.check_front = False

        if not self.rotate_sign and self.rotate_sign_old:
            if not self.check_front and navi_size:
                self.navi = self.navi[1:]
        elif stop_heigth and self.stop_height_old and stop_heigth > self.stop_height_old:
            if navi_size and self.navi[0] == 'G':
                self.navi = self.navi[1:]

        rospy.loginfo(f'error_stop = {error_stop}, error_go = {error_go}')
        rospy.loginfo(f'rotate sign = {self.rotate_sign} / check_front = {self.check_front}')
        rospy.loginfo(f'navi = {self.navi}')

        if stop_heigth != None:
            self.stop_height_old = stop_heigth
            rospy.loginfo(f'stop_height = {stop_heigth}')

        self.error_stop_old = error_stop
        self.error_go_old = error_go
        self.rotate_sign_old = self.rotate_sign



    def write_video(self):
        for i in range(len(self.detector.video_data)):
            self.detector.video_writer.write(self.detector.video_data[i])

    def camera_callback(self, msg):
        # if not self.is_running:
        #     return
        # direction = self.detector.get_direction(message=msg, line_color='yellow', tol=15)
        # self.motion_planner.move(direction)

        error_stop, error_go, stop_heigth = self.detector.get_direction_with_pid(message=msg, line_color=['green', 'yellow'], tol=50)
        self.examine_path(error_stop, error_go, stop_heigth)
        control_input = self.controller.PID_control(self.error)
        # control_input = self.controller.P_control(error)
        self.motion_planner.move_control(self.rotate_sign, control_input)

        self.publisher.publish(self.error)

    def D435_callback(self, msg):
        # print(msg.bounding_boxes)
        # if len(msg.bounding_boxes):
        #     print(msg.bounding_boxes[0].xmin, msg.bounding_boxes[0].xmax)
        #     print(320, (msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax)/2)

        error_stop, error_go, stop_heigth = self.find_bounding_box_stop_go(msg.bounding_boxes)
        rotate_direction, error = self.examine_path(error_stop, error_go, stop_heigth)
        control_input = self.controller.PID_control(error)
        self.motion_planner.move_control(rotate_direction, control_input)
        self.publisher.publish(error)


    def find_bounding_box_stop_go(self, bounding_boxes):
        error_stop = None
        error_go = None
        stop_heigth = None
        for box in bounding_boxes:
            cur = (box.xmin + box.xmax) / 2
            if box.Class == 'stop':
                error_stop = self.ref - cur
                stop_heigth = box.ymax - box.ymin
            elif box.Class == 'straight':
                error_go = self.ref - cur

            if error_stop and error_go:
                break

        return error_stop, error_go, stop_heigth

    def toggle_robot(self):
        while True:
            key_input = getch.getch()  # Get a single character input
            if key_input.lower() == 's':
                self.is_running = not self.is_running
                if self.is_running:
                    print("Robot started.")
                else:
                    print("Robot stopped.")

# NAVI = ['R', 'L', 'L', 'R', 'L', 'R', 'G', 'R'] 
# NAVI = ['R', 'L', 'L', 'R', 'L', 'L', 'R']
# NAVI = []
if __name__ == '__main__':
    NAVI = 'RLLRRRLR' # 아래 경로
    SIM_RATE = 30
    CAMERA_WIDTH = 680
    CAMERA_HEIGHT = 480
    follower = Follower(navi=NAVI, 
                        sim_rate=SIM_RATE, 
                        camera_width=CAMERA_WIDTH,
                        camera_height=CAMERA_HEIGHT
                )
    # thread = threading.Thread(target=follower.toggle_robot)
    # thread.start()
    follower.run()
