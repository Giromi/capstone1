#!/usr/bin/env python

# Python packages
import cv2
import numpy as np

# ROS packages (python)
import rospy
from cv_bridge import CvBridge

# Messages
from sensor_msgs.msg import Image

NOT_FOUND = 0
STRAIGHT = 1
LEFT = 2
RIGHT = 3

class LineDetector:
    def __init__(self, navi=[]):
        self.video_data = []
        self.video_writer = cv2.VideoWriter('../recordings/detection.avi', cv2.VideoWriter_fourcc(*'DIVX'), 30, (320, 240))
        self.bridge = CvBridge()

        self.detection = Image()
        self.publisher = rospy.Publisher('line_follower', Image, queue_size=1)
        self.rotate_direction = None
        self.error_stop_old = None
        self.error_go_old = None
        self.navi = navi
        self.rotate_direction_old = None
        self.check_front = False
        self.stop_height_old = None

    def read_image(self, message: Image):
        '''
        Reads ROS message and turns it into numpy array.
        It also applies a Gaussian blur and turns the blurred image into HSV color format.

        Inputs
        ---
            message: sensor_msgs.msg.Image
                Sensor image message
        '''

        self.image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        self.blurred = cv2.GaussianBlur(self.image, ksize=(3, 3), sigmaX=.1, sigmaY=.1)
        self.blurred_hsv = cv2.cvtColor(self.blurred, cv2.COLOR_BGR2HSV)
        
        self.height, self.width, _ = self.image.shape

    def set_lower_upper(self, line_color):
        lower = np.empty(0)
        upper = np.empty(0)

        if 'red' in line_color:
            lower = np.append(lower, np.array([0, 100, 100]))
            upper = np.append(upper, np.array([10, 255, 255]))
        if 'black' in line_color:
            lower = np.append(lower, np.array([0, 0, 0]))
            upper = np.append(upper, np.array([179, 20, 155]))
        if 'green' in line_color:
            lower = np.append(lower, np.array([35, 40, 40]))
            upper = np.append(upper, np.array([85, 255, 255]))
        if 'yellow' in line_color:
            lower = np.append(lower, np.array([20, 100, 100]))
            upper = np.append(upper, np.array([30, 255, 255]))

        lower = lower.reshape(2,3)
        upper = upper.reshape(2,3)
        return lower[0, :], lower[1, :], upper[0, :], upper[1, :]

    def get_direction_with_pid(self, message=None, line_color=[], tol=10.):
        if message:
            self.read_image(message)

        # 색상 범위 설정
        lower_stop, lower_go, upper_stop, upper_go = self.set_lower_upper(line_color)

        # 마스크 생성
        mask_stop = cv2.inRange(self.blurred_hsv, lower_stop, upper_stop)
        mask_go = cv2.inRange(self.blurred_hsv, lower_go, upper_go)

        # 중앙에 있는 라인의 위치 계산
        search_y = int(self.height * 2 / 5)
        mask_stop[:search_y, ] = 0
        mask_go[:search_y, ] = 0

        moments_stop = cv2.moments(mask_stop)
        moments_go = cv2.moments(mask_go)

        error_stop = None
        rect_stop = None
        try:
            cx_stop = int(moments_stop['m10'] / moments_stop['m00'])
            
            contours_stop, _ = cv2.findContours(mask_stop, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
            
            rect_stop = cv2.boundingRect(max(contours_stop, key=np.size))
            

            # stop bounding box
            cv2.rectangle(self.image, (rect_stop[0], rect_stop[1]), (rect_stop[0] + rect_stop[2], rect_stop[1] + rect_stop[3]), (0, 0, 255), 2)
            cv2.putText(self.image, 'Stop line', (rect_stop[0] - 2, rect_stop[1] - 8), cv2.FONT_HERSHEY_DUPLEX, .4, (0, 0, 255))

            

            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)
            error_stop = self.width/2 - cx_stop

        except ZeroDivisionError:
            cv2.putText(self.image, '[WARNING] No stop found', (int(self.width / 5), 20), cv2.FONT_HERSHEY_DUPLEX, .5, (0, 0, 255))

            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)
            #
            # rospy.logwarn('No stop found')
        error_go = None
        try :
            cx_go = int(moments_go['m10'] / moments_go['m00'])

            contours_go, _ = cv2.findContours(mask_go, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            rect_go = cv2.boundingRect(max(contours_go, key=np.size))

            # go bounding box
            cv2.rectangle(self.image, (rect_go[0], rect_go[1]), (rect_go[0] + rect_go[2], rect_go[1] + rect_go[3]), (0, 255, 0), 2)
            cv2.putText(self.image, 'Go line', (rect_go[0] - 2, rect_go[1] - 8), cv2.FONT_HERSHEY_DUPLEX, .4, (0, 255, 0))
        
            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)
            error_go = self.width/2 - cx_go
        
        
        except ZeroDivisionError:
            cv2.putText(self.image, '[WARNING] No go found', (int(self.width / 5), 20), cv2.FONT_HERSHEY_DUPLEX, .5, (0, 0, 255))

            self.detection = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
            self.publisher.publish(self.detection)

            self.video_data.append(self.image)

            rospy.logwarn('No go found')
        


        error = error_go if error_stop == None else error_stop
        if error == None :
            error = 0

        navi_size = len(self.navi)

        if self.rotate_direction == None and error_stop != None:
            self.rotate_direction = 0

        if error_stop == None and self.error_stop_old != None \
            and navi_size:
            rospy.logwarn('rotate flag')
            if self.navi[0] == 'L':
                self.rotate_direction = 1
            elif self.navi[0] == 'R':
                self.rotate_direction = -1

        if not self.check_front and error_go and self.error_go_old and abs(error_go) <= 100:
            if abs(error_go) > abs(self.error_go_old):
                self.check_front = False
            elif self.rotate_direction == 1 or self.rotate_direction == -1:
                rospy.logwarn('stop rotate')
                self.check_front = True
                error = 0
        elif self.check_front and self.rotate_direction and error_go and self.error_go_old: 
            if abs(error_go) <= abs(self.error_go_old) and 80 <= abs(error_go) <= 100:
                self.rotate_direction = None
                self.check_front = False


        # elif self.rotate_direction == -1 and error_go != None and  error_go >= 100:
        #     self.rotate_direction = None
        

        if not self.rotate_direction and self.rotate_direction_old:
            if not self.check_front and navi_size:
               self.navi.pop(0)
        elif rect_stop and self.stop_height_old and rect_stop[3] > self.stop_height_old:
            if navi_size and self.navi[0] == 'G' :
               self.navi.pop(0)

        rospy.loginfo(f'error_stop = {error_stop}, error_go = {error_go}')
        rospy.loginfo(f'flag = {self.rotate_direction} / check_front = {self.check_front}')
        rospy.loginfo(f'navi = {self.navi}')
        if rect_stop != None:
            self.stop_height_old = rect_stop[3]
            rospy.loginfo(f'{rect_stop[0]}, {rect_stop[1]}, {rect_stop[2]}, {rect_stop[3]}')
        self.error_stop_old = error_stop
        self.error_go_old = error_go
        self.rotate_direction_old = self.rotate_direction
        return self.rotate_direction, error

        # 로봇을 제어하기 위한 방향 계산
        # if abs(error) <= tol:  # 직진
        #     error = 0
        # elif error < 0:  # 왼쪽
        # else:  # 오른쪽
