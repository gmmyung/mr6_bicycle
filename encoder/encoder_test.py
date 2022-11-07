import RPi.GPIO as GPIO
import time       
import math

angular_velocity = 0

class LowPassFilter(object):
    def __init__(self, cut_off_freqency, ts):
        self.ts = ts
        self.cut_off_freqency = cut_off_freqency
        self.tau = self.get_tau()
        
        self.prev_data = 0.
        
    def get_tau(self):
        return 1/(2*math.pi*self.cut_off_freqency)
    
    def Filter(self,data):
        val = (self.ts * data + self.tau * self.prev_data) / (self.tau + self.ts)
        self.prev_data = val
        return val

class Read_encoder():
    def __init__(self,encoder_pin):
        self.encoder_pin = encoder_pin
        self.sample_rate = 5 #Hz
        self.n_hole = 20
        self.lpf = LowPassFilter(1.0,self.sample_rate)
        
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_pin, GPIO.IN)
        
    def mode_1(self,T):
        global angular_velocity
        total_time = 0
        now_time = time.time()
        blocked = False
        while total_time < T:
            encoder_read = GPIO.input(self.encoder_pin)
            if encoder_read:
                if not blocked:
                    prev_time = now_time
                    now_time = time.time()
                    delta_time = now_time - prev_time
                    angular_velocity = (360 / self.n_hole) / delta_time
                    angular_velocity = self.lpf.Filter(angular_velocity)
                    print(angular_velocity)
                    total_time += delta_time
                    
                blocked = True
                
            else:
                blocked = False

    def mode_1_infinite(self):
        global angular_velocity
        total_time = 0
        now_time = time.time()
        blocked = False
        while True:
            encoder_read = GPIO.input(self.encoder_pin)
            if encoder_read:
                if not blocked:
                    prev_time = now_time
                    now_time = time.time()
                    delta_time = now_time - prev_time
                    angular_velocity = (360 / self.n_hole) / delta_time
                    angular_velocity = self.lpf.Filter(angular_velocity)
                    print(angular_velocity)
                    
                blocked = True
                
            else:
                blocked = False
            
            
    def mode_2(self,T):
        global angular_velocity
        total_time = 0
        
        while total_time < T:
            sample_period = 1 / self.sample_rate
            start_time = time.time()
            current_time = time.time()
            end_time = start_time + sample_period

            cnt = 0
            prev_state = False
            
            while current_time < end_time:
                cur_state = GPIO.input(self.encoder_pin)
                
                if prev_state != cur_state:
                    cnt+=1
                
                if cur_state:
                    prev_state = True
                
                else:
                    prev_state = False
                    
                current_time = time.time()
            
            angular_velocity = (cnt/2)*(360/self.n_hole) / sample_period #deg/s
            angular_velocity = self.lpf.Filter(angular_velocity)
            print(angular_velocity)
            total_time += sample_period
            
    def mode_2_infinite(self):
        global angular_velocity
        total_time = 0
        while True:
            sample_period = 1 / self.sample_rate
            start_time = time.time()
            current_time = time.time()
            end_time = start_time + sample_period

            cnt = 0
            prev_state = False
            
            while current_time < end_time:
                cur_state = GPIO.input(self.encoder_pin)
                
                if prev_state != cur_state:
                    cnt+=1
                
                if cur_state:
                    prev_state = True
                
                else:
                    prev_state = False
                    
                current_time = time.time()
            
            angular_velocity = (cnt/2)*(360/self.n_hole) / sample_period #deg/s
            angular_velocity = self.lpf.Filter(angular_velocity)
            print(angular_velocity)
            total_time += sample_period

'''
read = Read_encoder(16)
read.mode_1(500)
'''