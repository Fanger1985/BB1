import aiohttp  # Asynchronous HTTP client
import asyncio  # For asynchronous operations
import time  # Timing and delays
import RPi.GPIO as GPIO  # For controlling GPIO pins

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"  # Replace with your ESP32's IP address

# Set up GPIO pins
GPIO.setmode(GPIO.BCM)  # Broadcom pin-numbering scheme
GPIO.setup(12, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)

# Create PWM instances on pins 12 and 13
left_servo = GPIO.PWM(12, 50)  # GPIO 12 for left servo, 50Hz
right_servo = GPIO.PWM(13, 50)  # GPIO 13 for right servo, 50Hz

left_servo.start(7.5)  # Initialize with neutral position
right_servo.start(7.5)

def move_servos(emotion):
    if emotion == "wiggle":
        for _ in range(5):
            left_servo.ChangeDutyCycle(5)  # Move to min position
            right_servo.ChangeDutyCycle(10)  # Move to max position
            time.sleep(0.2)
            left_servo.ChangeDutyCycle(10)  # Move to max position
            right_servo.ChangeDutyCycle(5)  # Move to min position
            time.sleep(0.2)
        # Reset servos to neutral position to prevent jitter
        left_servo.ChangeDutyCycle(7.5)
        right_servo.ChangeDutyCycle(7.5)
        time.sleep(0.5)
        # Stop the servos to prevent jittering
        left_servo.ChangeDutyCycle(0)
        right_servo.ChangeDutyCycle(0)

async def send_http_get(endpoint):
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(esp32_base_url + endpoint) as response:
                if response.status == 200:
                    print(f"GET request to {endpoint} succeeded")
                else:
                    print(f"GET request to {endpoint} failed with status {response.status}")
    except Exception as e:
        print(f"Error sending HTTP GET request: {e}")

async def control_robot(command):
    endpoints = {
        "forward": "/forward",
        "backward": "/backward",
        "left": "/left",
        "right": "/right",
        "stop": "/stop",
        "explore": "/explore",
        "dance": "/dance",
        "auto": "/auto",
    }
    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

async def figure_8_with_wiggles():
    await control_robot("forward")
    move_servos("wiggle")
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("left")
    move_servos("wiggle")
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("forward")
    move_servos("wiggle")
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("right")
    move_servos("wiggle")
    await asyncio.sleep(1)
    await control_robot("stop")

async def main():
    print("Starting Pi control script for the ESP32-based robot")
    await figure_8_with_wiggles()
    print("Pi control script execution complete.")

def cleanup_gpio():
    left_servo.stop()
    right_servo.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cleanup_gpio()
