import RPi.GPIO as GPIO
import time

# Define GPIO pins
#TRIG = 6  # Pin connected to TRIG #20
#ECHO = 5  # Pin connected to ECHO #21

TRIG_RIGHT = 6
ECHO_RIGHT = 5

TRIG_LEFT = 20
ECHO_LEFT = 21

#

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_RIGHT, GPIO.OUT)
GPIO.setup(ECHO_RIGHT, GPIO.IN)
GPIO.output(TRIG_RIGHT, False)

GPIO.setup(TRIG_LEFT, GPIO.OUT)
GPIO.setup(ECHO_LEFT, GPIO.IN)
GPIO.output(TRIG_LEFT, False)


time.sleep(1)  # Let the sensor settle

def measure_distance_left():
    # Send a short pulse to trigger the ultrasonic burst
    
    GPIO.output(TRIG_LEFT, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_LEFT, False)

    # Wait for the echo response
    while GPIO.input(ECHO_LEFT) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO_LEFT) == 1:
        pulse_end = time.time()
    
    # Calculate distance based on the time difference
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Sound speed in air (34300 cm/s) divided by 2
    distance = round(distance, 2)  # Round to two decimal places
  
    return distance




def measure_distance_right():
    # Send a short pulse to trigger the ultrasonic burst
    
    GPIO.output(TRIG_RIGHT, True)
    time.sleep(0.00001)  # 10 microseconds
    GPIO.output(TRIG_RIGHT, False)

    # Wait for the echo response
    while GPIO.input(ECHO_RIGHT) == 0:
        pulse_start = time.time()
    
    while GPIO.input(ECHO_RIGHT) == 1:
        pulse_end = time.time()
    
    # Calculate distance based on the time difference
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Sound speed in air (34300 cm/s) divided by 2
    distance = round(distance, 2)  # Round to two decimal places
  
    return distance

def main():
    while True:
        dist_left = measure_distance_left()
        dist_right = measure_distance_right()
        print(f"Distance Right: {dist_right} cm")
        print(f"Distance Left: {dist_left} cm")
        time.sleep(0.25)  # Wait 1 second between measurements

if __name__ == '__main__':
    main()