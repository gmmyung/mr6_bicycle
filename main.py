import multiprocessing as mp
#from encoder import Encoder
from gyro import Gyro
#from servo_motor import Servo_motor
import time
from threading import Thread
from graph import graph

class Main:
    
    def clear_q(self, q):
        while not q.empty():
            q.get()
        print("clear")

    def __init__(self, command_q):
        self.frequency = 20 #5Hz

        self.encoder_q = mp.Queue()
        #self.e = Encoder(36, self.encoder_q, sample_rate = self.frequency)
        #self.encoder_proc = mp.Process(target=self.e.run)


        self.gyro_q = mp.Queue()
        self.g = Gyro(self.gyro_q, sample_rate = self.frequency)
        self.gyro_proc = mp.Process(target=self.g.run_pseudo)
        
        self.actuator_q = mp.Queue()
        #self.servo_q = mp.Queue
        #self.s = Servo_motor(31) #board num 31 is bcm num 6

        self.command_q = command_q
        
        #self.clear_q(self.gyro_q)
        
        print("main initialized")
        

    def calc(self): #override this method to calculate actuator movement.
        #servo_q.put(something)
        return
    
    def run(self):
        gr = graph(1, (30,), (600, 1000))
        #self.encoder_proc.start()
        self.gyro_proc.start()
        Thread(target=gr.run, args=(self.gyro_q,)).run()

        while True:
            self.calc()
            #self.s.run(self.servo_q.get())
            #print(i, end = " ")
            #print(self.encoder_q.get())
            #print(self.gyro_q.get())
            #if self.command_q.get() == "end":
            #    print("terminated")
            #    return
            #time.sleep(1 / self.frequency)
                

def command(q):
    print("command start")
    while True:
        q.put(input(">>"))


if __name__ == "__main__":
    command_q = mp.Queue()
    mr6 = Main(command_q)
    #Thread(target=command, args=(command_q,)).start()
    
    mr6.run()
    


