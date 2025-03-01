
import rospy
import numpy as np

class Controller:
    def __init__(self, Kp=0., Ki=0., Kd=0., T=1.0):
        self.Kp = Kp  # P 제어기의 게인
        self.Ki = Ki
        self.Kd = Kd
        self.T = T
        self.e_k = np.array([0.0, 0.0, 0.0])
        self.u_k = np.array([0.0, 0.0])

    # def P_control(self, cur_error):
    #     P = self.Kp * self.e_k[0] 
    #     return P
    #
    # def I_control(self, cur_error):
    #     I = self.control_input + self.Ki * self.T * self.e_k[0]
    #     self.integral_error += self.e_k[0]
    #     I = self.Ki * self.integral_error
    #     return I
    #
    # def D_control(self, cur_error):
    #     self.old_error = self.e_k[0]
    #     return D

    def PI(self, cur_error):
        return self.P_control(self.e_k[0])    \
                + self.I_control(self.e_k[0])
    #
    # def PD(self, cur_error):
    #     return self.P_control(self.e_k[0])    \
    #             + self.D_control(self.e_k[0])

    def PID_control(self, cur_error):
        rospy.loginfo(f'e_k = {self.e_k} / u_k = {self.u_k}')
        self.e_k = np.roll(self.e_k, 1)
        self.u_k = np.roll(self.u_k, 1)
        self.e_k[0] = cur_error
        self.u_k[0] = self.u_k[1] + \
                      (self.Kp + self.Ki + self.Kd / self.T) * self.e_k[0] + \
                      (self.Kp + 2 * self.Kd / self.T) * self.e_k[1] + \
                      (self.Kd / self.T) * self.e_k[2]
        rospy.loginfo(f'e_k = {self.e_k} / u_k = {self.u_k}')
        return self.u_k[0]
        # return self.Kp * self.e_k[0] 




