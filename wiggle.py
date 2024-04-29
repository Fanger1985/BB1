import cv2
import pigpio
import time

# Setup the GPIO for servo control
pi = pigpio.pi()
servo_pin = 18  # Change to your GPIO pin connected to the servo

# Set servo position function
def set_servo_position(angle):
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
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))
            
            if len(faces) > 0:
                print("Face detected! Wiggling the servo...")
                set_servo_position(90)  # Move servo to 90 degrees
                time.sleep(1)  # Pause for a second
                set_servo_position(45)  # Move servo to 45 degrees
                time.sleep(1)  # Pause for a second
            else:
                set_servo_position(0)  # Reset servo position when no face is detected
        else:
            print("Failed to capture frame")
        
        time.sleep(0.1)  # Short delay to ease on the CPU
finally:
    camera.release()
    pi.stop()  # Cleanup the GPIO
    print("Cleanup complete, camera and GPIO released.")

