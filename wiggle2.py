import cv2
import pigpio
import time

# Setup the GPIO for servo control
pi = pigpio.pi()
servo_pin1 = 12  # GPIO pin for the first servo
servo_pin2 = 13  # GPIO pin for the second servo

# Set servo position function
def set_servo_position(servo_pin, angle):
    pulsewidth = int((angle * 11.11) + 500)  # Convert angle to pulse width
    pi.set_servo_pulsewidth(servo_pin, pulsewidth)

# Load OpenCV pre-trained Haar Cascade for face detection
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Initialize the camera
camera = cv2.VideoCapture(0)  # Assuming the first camera

try:
    while True:
        ret, frame = camera.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=7, minSize=(30, 30))
            
            if len(faces) > 0:
                print("Face detected! Wiggling the servos...")
                set_servo_position(servo_pin1, 90)  # Move first servo to 90 degrees
                set_servo_position(servo_pin2, 90)  # Move second servo to 90 degrees
                time.sleep(1)  # Pause for a second
                set_servo_position(servo_pin1, 45)  # Move first servo to 45 degrees
                set_servo_position(servo_pin2, 45)  # Move second servo to 45 degrees
                time.sleep(1)  # Pause for a second
            else:
                print("No face detected.")
                set_servo_position(servo_pin1, 0)  # Reset first servo position when no face is detected
                set_servo_position(servo_pin2, 0)  # Reset second servo position when no face is detected
        else:
            print("Failed to capture frame")
        
        time.sleep(0.1)  # Short delay to ease on the CPU
finally:
    camera.release()
    pi.stop()  # Cleanup the GPIO
    print("Cleanup complete, camera and GPIO released.")
