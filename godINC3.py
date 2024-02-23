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

# Load the pre-trained model and class labels for object detection
net = cv2.dnn.readNetFromCaffe('path_to/MobileNetSSD_deploy.prototxt', 'path_to/MobileNetSSD_deploy.caffemodel')
classes = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

# Function to capture and process images from USB camera
def capture_and_process_image():
    ret, frame = camera.read()
    if ret:
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

        # Pass the blob through the network and obtain the detections and predictions
        net.setInput(blob)
        detections = net.forward()

        # Loop over the detections
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.2:  # Threshold to filter weak detections
                idx = int(detections[0, 0, i, 1])
                detected_class = classes[idx]
                
                # Do something based on detected object, e.g., stop if a person is detected
                if detected_class == "person":
                    print("Woah, there's a person ahead! Let's stop.")
                    # Here you can call a function to stop or change the bot's direction

# Function to drive a single motor (placeholder for your implementation)
def drive_motor(motor_forward_pin, motor_reverse_pin, direction):
    pass  # Add your motor control code here

# Function to drive both motors (placeholder for your implementation)
def drive_motors(direction, duration):
    pass  # Add your motor control code here

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
