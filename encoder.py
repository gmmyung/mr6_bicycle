from distutils.util import change_root
import RPi.GPIO as GPIO
import time
import math
import multiprocessing as mp
import random

class Encoder:

    def __init__(self, encoder_pin, q, mode = 1, sample_rate = 5, hole_number = 20):
        GPIO.setmode(GPIO.BOARD)
        self.mode = mode #1, 2 for each mode1, mode2
        self.encoder_pin = encoder_pin 
        self.sample_rate = sample_rate
        self.hole_number = hole_number
    
        GPIO.setup(self.encoder_pin, GPIO.IN)

        self.q = q
    
    def change_counter(self):
        encoder_read = GPIO.input(self.encoder_pin)
        if encoder_read == GPIO.input(self.encoder_pin):
            return False
        return True
    
    def mode1(self):
        time1 = time.time()
        cnt = 0
        while time.time() - time1 <= 1 / self.sample_rate:
            if self.change_counter():
                cnt += 1
        return math.pi * cnt * self.sample_rate / self.hole_number

    def mode2(self):
        time1 = time.time()
        if self.change_counter():
            if self.change_counter():
                return 2 * math.pi / ((time.time() - time1) * self.hole_number)
                
    def run(self):
        print("encoder sensing start")
        print(self.mode)
        if self.mode == 1:
            while True:
                self.q.put(self.mode1())
                #print(self.q.get())
                #print("running")
            
        elif self.mode == 2:
            while True:
                self.q.put(self.mode2())
    
    def run_pseudo(self):
        d = 0
        while True:
            d += (random.random() - 0.5)
            self.queue.put(d)
            time.sleep(1 / self.sample_rate)
        
        

if __name__ == "__main__":
    test_q = mp.Queue()
    test_e = Encoder(36, test_q, mode=1, sample_rate = 1)
    test_e.run()
    