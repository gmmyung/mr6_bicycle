import RPi.GPIO as GPIO
import time

class Servo_motor:

    def __init__(self, servo_pin):
        GPIO.setmode(GPIO.BOARD) #board based gpio pin number
        GPIO.setup(servo_pin, GPIO.OUT)

        self.pwm = GPIO.PWM(servo_pin, 50) #frequency is 50Hz
        self.pwm.start(0) #initialization

    def turn(self, angle):
        duty = ((angle + 90) / 18) + 2
        self.pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        
