from main import main
import time

class pid_control(main):

    def __init__(self, )

    def calc(self):
        error = target_degree - pre_servo_degree

time.time


global pre_servo_degree
global time_prev
global error_prev
global error

error = target_degree-pre_servo_degree
de = error-error_prev 
dt=time.time()-time_prev

angle = kp*error + kd*de/dt + ki*error*dt

error_prev = error
time_prev = time.time()

servo1.turn_to_deg(angle,  1)    