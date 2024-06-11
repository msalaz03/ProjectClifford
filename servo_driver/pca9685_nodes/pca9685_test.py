#!/usr/bin/env python

import time
import board
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

#Create the I2C bus interface.
i2c = board.I2C()

#servo specifications
USMIN = 600
USMAX = 2400
USNEUTREL = 1500
SERVO_FREQ = 60
SERVO_MIN = 150
SERVO_MAX = 600
ACT_RANGE = 180

#Create a simple PCA9685 class instance
pca = PCA9685(i2c)

# Set the PWM frequency to 60hz.
pca.frequency = 60

#Set the PWM duty cycle for channel zero to 50%. duty_cycle is 16 bits to match other PWM objects
#but the PCA9685 will only actually give 12 bits of resolution
pca.channels[0].duty_cycle = 0x7FFF #look more into this

servo0 = servo.Servo(pca.channels[0],ACT_RANGE,USMIN,USMAX)


def test_servos():
    for i in range (180):
        servo0.angle = i
        time.sleep(0.03)
    
    for i in range (180):
        servo0.angle = 180 - i
        time.sleep(0.03)

def main(args = None):
    print("Running 'test_servos' script")
    test_servos()

if __name__ == '__main__':
    main()

