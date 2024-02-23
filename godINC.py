import RPi.GPIO as GPIO
import time
import random
import cv2
from picamera import PiCamera
from picamera.array import PiRGBArray

# Motor setup
motor_pins = [17, 18, 27, 22]  # Replace with your motor GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pins, GPIO.OUT)

# Ultrasonic sensor setup
trigger_pin = 23  # Replace with your trigger pin number
echo_pin = 24     # Replace with your echo pin number
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

# Camera setup
camera = PiCamera()
camera.resolution = (640, 480)
rawCapture = PiRGBArray(camera, size=(640, 480))

# Function to drive motors
def drive_motors(direction, duration):
    if direction == 'forward':
        # Set your motor pins high/low to move forward
        pass
    elif direction == 'turn':
        # Set your motor pins high/low to turn
        pass
    time.sleep(duration)
    # Set all motor pins low to stop
    GPIO.output(motor_pins, GPIO.LOW)

# Function to check for obstacles
def check_for_obstacles():
    # Send out an ultrasonic pulse
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = time.time()

    # Save start time
    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    # Save arrival time
    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    # Time difference and distance calculation
    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound in cm/s

    return distance

# Function to capture and process images
def capture_and_process_image():
    camera.capture(rawCapture, format="bgr")
    image = rawCapture.array

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Look for interesting features or objects
    # For simplicity, let's detect edges which could be considered as "interesting points"
    edges = cv2.Canny(gray, 100, 200)

    # Check if we have a significant number of edges
    if cv2.countNonZero(edges) > 10000:  # Adjust this threshold as needed
        print("Found something interesting!")
        # Add your logic here to handle an interesting point (e.g., take a picture, store the location, etc.)

    rawCapture.truncate(0)

# Main loop
try:
    while True:
        distance = check_for_obstacles()
        if distance > 20:  # If no obstacle closer than 20cm, move forward
            drive_motors('forward', 2)
        else:
            drive_motors('turn', random.uniform(0.5, 1.5))  # Randomly turn if an obstacle is detected

        capture_and_process_image()

except KeyboardInterrupt:
    print("Program stopped by User")
    GPIO.cleanup()
