import RPi.GPIO as GPIO
import time
import random
import cv2

# Define motor GPIO pins
motor1_forward_pin = 17
motor1_reverse_pin = 18
motor2_forward_pin = 27
motor2_reverse_pin = 22

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)
GPIO.setup([motor1_forward_pin, motor1_reverse_pin, motor2_forward_pin, motor2_reverse_pin], GPIO.OUT, initial=GPIO.LOW)

# Ultrasonic sensor setup for both front and back
front_trigger_pin = 23
front_echo_pin = 24
back_trigger_pin = 25  # Use different pins for the back sensor
back_echo_pin = 8      # Use different pins for the back sensor
GPIO.setup([front_trigger_pin, front_echo_pin, back_trigger_pin, back_echo_pin], GPIO.OUT, initial=GPIO.LOW)
GPIO.setup([front_echo_pin, back_echo_pin], GPIO.IN)

# Set up USB camera with OpenCV
camera = cv2.VideoCapture(0)  # 0 is usually the default ID for the first cam

# Function to capture and process images from USB camera
def capture_and_process_image():
    ret, frame = camera.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        if cv2.countNonZero(edges) > 10000:  # Tweak this threshold as needed
            print("Found something interesting, dude!")

# Function to drive a single motor
def drive_motor(motor_forward_pin, motor_reverse_pin, direction):
    # Same as before, no changes needed here

# Function to drive both motors
def drive_motors(direction, duration):
    # Same as before, no changes needed here

# Function to check for obstacles with a specific sensor
def check_for_obstacles(trigger_pin, echo_pin):
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2  # Speed of sound in cm/s

    return distance

# Main loop and cleanup
try:
    while True:
        front_distance = check_for_obstacles(front_trigger_pin, front_echo_pin)
        back_distance = check_for_obstacles(back_trigger_pin, back_echo_pin)

        if front_distance > 20 and back_distance > 20:  # Clear path ahead and behind
            drive_motors('forward', 2)
        elif front_distance <= 20 and back_distance > 20:  # Obstacle in front, clear behind
            drive_motors('reverse', 2)  # Maybe reverse a bit
        elif back_distance <= 20 and front_distance > 20:  # Obstacle behind, clear in front
            drive_motors('forward', 2)  # Keep moving forward
        else:
            drive_motors('turn', random.uniform(0.5, 1.5))  # Trapped! Time to turn.

        capture_and_process_image()

except KeyboardInterrupt:
    print("Program stopped by User, peace out!")
    camera.release()  # Release the camera
    cv2.destroyAllWindows()  # Close any OpenCV windows
    GPIO.cleanup()  # Clean up GPIO
