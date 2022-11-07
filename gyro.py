import time
import math
#import mpu6050
import multiprocessing as mp
import random

class Gyro:
    def __init__(self, queue, sample_rate=5):
        self.queue = queue
        self.sample_rate = sample_rate
        
        # Sensor initialization
        """
        self.mpu = mpu6050.MPU6050()
        self.mpu.dmpInitialize()
        self.mpu.setDMPEnabled(True)
        self.mpu.setRate(int(1000 / self.sample_rate - 1))

        # get expected DMP packet size for later comparison
        self.packetSize = self.mpu.dmpGetFIFOPacketSize()
        """
    """
    def run(self):
        while True:
            # Get INT_STATUS byte
            mpuIntStatus = self.mpu.getIntStatus()
        
            if mpuIntStatus >= 2: # check for DMP data ready interrupt (this should happen frequently) 
                # get current FIFO count
                fifoCount = self.mpu.getFIFOCount()
                
                # check for overflow (this should never happen unless our code is too inefficient)
                if fifoCount == 1024:
                    # reset so we can continue cleanly
                    self.mpu.resetFIFO()
                    print('FIFO overflow!')
                    
                # wait for correct available data length, should be a VERY short wait
                fifoCount = self.mpu.getFIFOCount()
                while fifoCount < self.packetSize:
                    fifoCount = self.mpu.getFIFOCount()
                
                result = self.mpu.getFIFOBytes(self.packetSize)
                q = self.mpu.dmpGetQuaternion(result)
                g = self.mpu.dmpGetGravity(q)
                ypr = self.mpu.dmpGetYawPitchRoll(q, g)

                # track FIFO count here in case there is > 1 packet available
                # (this lets us immediately read more without waiting for an interrupt)        
                fifoCount -= self.packetSize
    
                ans = [time.time(), ypr['yaw'] * 180 / math.pi, ypr['pitch'] * 180 / math.pi, ypr['roll'] * 180 / math.pi]
                self.queue.put(ans)
                print("raw", self.queue.get())
                #print(self.queue.size)
    """
    
    def run_pseudo(self):
        d = 0
        while True:
            d += (random.random() - 0.5)
            self.queue.put(d)
            time.sleep(1 / self.sample_rate)

if __name__ == '__main__':
    test_q = mp.Queue()
    test_gyro = Gyro(test_q, sample_rate = 100)
    gp = mp.Process(target=test_gyro.run_pseudo)
    print("start")
    gp.start()
    while True:
        print(test_q.get())
        time.sleep(1 / 100)