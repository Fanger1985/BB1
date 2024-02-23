import pigpio
import keyboard

# Initialize pigpio
pi = pigpio.pi()

# Servo GPIO pins
servo1_pin = 12
servo2_pin = 13

# Servo control parameters
angle_0 = 500  # Pulse width for 0 degrees
angle_180 = 2500  # Pulse width for 180 degrees
current_angle1 = angle_0
current_angle2 = angle_0

# Function to toggle servo angle
def toggle_servo(pin, current_angle):
    new_angle = angle_180 if current_angle == angle_0 else angle_0
    pi.set_servo_pulsewidth(pin, new_angle)
    return new_angle

# Main loop to listen for 'v' and 'n' keys
try:
    print("Press 'v' or 'n' to toggle servos. Press 'q' to quit.")
    while True:
        if keyboard.is_pressed('v'):  # If 'v' is pressed, toggle servo 1
            current_angle1 = toggle_servo(servo1_pin, current_angle1)
            while keyboard.is_pressed('v'):  # Wait for the key to be released
                pass

        if keyboard.is_pressed('n'):  # If 'n' is pressed, toggle servo 2
            current_angle2 = toggle_servo(servo2_pin, current_angle2)
            while keyboard.is_pressed('n'):  # Wait for the key to be released
                pass

        if keyboard.is_pressed('q'):  # If 'q' is pressed, quit
            print("Quitting...")
            break

finally:
    # Clean up
    pi.set_servo_pulsewidth(servo1_pin, 0)  # Turn off servo 1
    pi.set_servo_pulsewidth(servo2_pin, 0)  # Turn off servo 2
    pi.stop()
