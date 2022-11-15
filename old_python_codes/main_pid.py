from main import Main
import time

class pid_control(Main):

    def __init__(self, kp, ki, kd):
        self.kp = 0.3
        self.ki = 0.2
        self.kd = 0.4

    def calc(self): #pid 제어 계산
        time_now = time.time()
        target_degree = self.calc_degree()

        error = target_degree - pre_servo_degree
        de = error - error_prev
        dt = time_now - time_prev

        angle = self.kp*error + self.kd*de/dt + self.ki*error*dt
        pre_servo_degree = target_degree
        error_prev = error
        time_prev = time_now
    
    def calc_degree(self): #역학식 바탕으로 target_degree 계산
        target_degree = 11
        return target_degree