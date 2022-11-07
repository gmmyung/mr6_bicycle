import time
from gyro_modulized import Gyro
import threading

new_gyro = Gyro()

t = threading.Thread(target=new_gyro.gyro_main)
t.start()

while True:
    print(time.time())
    print(new_gyro.get_data())
    time.sleep(1)
