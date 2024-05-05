

class Controller:
    def __init__(self, Kp=0., Ki=0., Kd=0.):
        self.Kp = Kp  # P 제어기의 게인
        self.Ki = Ki
        self.Kd = Kd
        self.old_error = 0.0 
        self.integral_error = 0.0 

    def P_control(self, cur_error):
        P = self.Kp * cur_error 
        return P

    def I_control(self, cur_error):
        self.integral_error += cur_error
        I = self.Ki * self.integral_error
        return I

    def D_control(self, cur_error):
        D = self.Kd * (cur_error - self.old_error)
        self.old_error = cur_error
        return D

    def PI(self, cur_error):
        return self.P_control(cur_error)    \
                + self.I_control(cur_error)

    def PD(self, cur_error):
        return self.P_control(cur_error)    \
                + self.D_control(cur_error)

    def PID(self, cur_error):
        return self.P_control(cur_error)    \
                + self.I_control(cur_error) \
                + self.D_control(cur_error)




