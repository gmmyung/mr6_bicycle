import RPi.GPIO as GPIO
import time
import encoder.encoder_test as Encoder
import threading
import random

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#gyro sensor SCL=17, SDA=27
encoder_pin = 16
max_voltage = 25
DC_limit = 7

class servo_motor():
    def __init__(self, Pin):
        self.Pin = Pin #servo pin
        
        GPIO.setup(self.Pin, GPIO.OUT)

        self.pwm = GPIO.PWM(self.Pin, 50) # Frequency is 50 Hz
        self.pwm.start(0) #initialization

    """ turn the servo to 'angle' degree where 'angle': -90 deg ~ 90 deg """
    def turn_to_deg(self, angle = 0,  sleep_time = 1):
        duty = ((angle + 90) / 18) + 2
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(sleep_time)
        self.pwm.ChangeDutyCycle(0)

class DC_motor():
    def __init__(self, Ena, In1, In2):
        global max_voltage
        global DC_limit
        
        self.Ena = Ena # Enable
        self.In1 = In1 # Input1
        self.In2 = In2 # Input2
        self.max_voltage = max_voltage
        self.DC_limit = DC_limit
        
        GPIO.setup(self.Ena, GPIO.OUT)
        GPIO.setup(self.In1, GPIO.OUT)
        GPIO.setup(self.In2, GPIO.OUT)

        self.pwm = GPIO.PWM(self.Ena, 100) # Frequency is 100 Hz
        self.pwm.start(0)

    """ trun the motor cw at 'speed' for 'duration' """
    
    def turn_cw(self, speed = 0):
        GPIO.output(self.In1, GPIO.LOW)
        GPIO.output(self.In2, GPIO.HIGH)
        speed_convert = speed*(self.DC_limit/self.max_voltage)
        self.pwm.ChangeDutyCycle(speed_convert)

    """ trun the motor ccw at 'speed' for 'duration' """
    def turn_ccw(self, speed = 0):
        GPIO.output(self.In1, GPIO.HIGH)
        GPIO.output(self.In2, GPIO.LOW)
        speed_convert = speed*(self.DC_limit/self.max_voltage)
        self.pwm.ChangeDutyCycle(speed_convert)

    """ stop the motor """
    def stop(self):
        self.pwm.ChangeDutyCycle(0)

####init####
motor1 = DC_motor(13,19,26)

servo1 = servo_motor(6)
servo1.turn_to_deg(20)
pre_servo_degree = 20
time_prev = time.time()
error_prev = 0
error = 0
#########


read = Encoder.Read_encoder(encoder_pin)

th1 = threading.Thread(target=read.mode_2_infinite)

def random_turn():
    motor_speed = 5*random.randrange(4,20)
    while True:
        motor1.turn_cw(motor_speed)
        #print(Encoder.angular_velocity)

th2 = threading.Thread(target=random_turn)

def PID_Handle(target_degree, kp,ki,kd):
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

th1.start()
th2.start()

th1.join()
th2.join()
