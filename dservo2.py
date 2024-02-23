import pigpio  # For controlling the GPIO pins
import keyboard  # For capturing keyboard inputs
import time  # For adding delays

# Fire up the pigpio engine
pi = pigpio.pi()

# Pin numbers for the servos, change these if yours are different
servo1_pin = 12
servo2_pin = 13

# Servo specs: pulse widths for 0 and 180 degrees
angle_0 = 500
angle_180 = 2500
current_angle1 = angle_0
current_angle2 = angle_0

# This little function is the heart of the show, flipping the servo angle
def toggle_servo(pin, current_angle):
    new_angle = angle_180 if current_angle == angle_0 else angle_0
    pi.set_servo_pulsewidth(pin, new_angle)  # Magic happens here, making the servo move
    return new_angle

# Main stage: listening for your commands
try:
    print("Rock 'n roll with 'v' or 'n'. Hit 'q' to drop the mic.")
    while True:
        # Toggle servo 1 with 'v'
        if keyboard.is_pressed('v'):
            current_angle1 = toggle_servo(servo1_pin, current_angle1)
            print("Servo 1 is jamming!")  # Feedback so you know it's grooving
            time.sleep(0.3)  # Chill for a bit to avoid going nuts with the toggling

        # Toggle servo 2 with 'n'
        if keyboard.is_pressed('n'):
            current_angle2 = toggle_servo(servo2_pin, current_angle2)
            print("Servo 2 is rocking out!")  # Letting you know servo 2 is on the move
            time.sleep(0.3)  # A little breather to prevent servo hysteria

        # Bail out with 'q'
        if keyboard.is_pressed('q'):
            print("Show's over, folks!")
            break

finally:
    # The cleanup crew: make sure we leave the stage tidy
    pi.set_servo_pulsewidth(servo1_pin, 0)  # Servo 1, take a rest
    pi.set_servo_pulsewidth(servo2_pin, 0)  # Servo 2, you too
    pi.stop()  # Shutting down the pigpio show
