import aiohttp  # Asynchronous HTTP client
import asyncio  # For asynchronous operations
import time  # Timing and delays
import gpiozero  # For controlling GPIO pins

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"  # Replace with your ESP32's IP address

# Initialize servos for the robot's "ears"
left_servo = gpiozero.Servo(12)  # GPIO 12
right_servo = gpiozero.Servo(13)  # GPIO 13

# Function to control servo positions to indicate emotions or add personality
def move_servos(emotion):
    if emotion == "wiggle":
        for _ in range(5):
            left_servo.min()
            right_servo.max()
            time.sleep(0.2)  # Short wiggle
            left_servo.max()
            right_servo.min()
            time.sleep(0.2)

# Asynchronous function to send HTTP GET requests to the ESP32
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

# Asynchronous function to control the robot
async def control_robot(command):
    # Map commands to endpoint paths
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

    # Ensure valid command
    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

# Asynchronous function for the figure-8 pattern with servo wiggles
async def figure_8_with_wiggles():
    # Define the figure-8 dance pattern with servo wiggles
    await control_robot("forward")
    move_servos("wiggle")  # Add servo wiggles
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("left")
    move_servos("wiggle")  # Continue servo wiggles
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("forward")
    move_servos("wiggle")  # Keep the wiggles going
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("right")
    move_servos("wiggle")  # Wrap up with more wiggles
    await asyncio.sleep(1)
    await control_robot("stop")

# Main asynchronous function for robot control and the figure-8 dance
async def main():
    print("Starting Pi control script for the ESP32-based robot")

    # Perform the figure-8 dance with servo wiggles
    await figure_8_with_wiggles()

    print("Pi control script execution complete.")

# Start the asynchronous event loop and run the main function
if __name__ == "__main__":
    asyncio.run(main())
