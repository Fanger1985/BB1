import cv2  # OpenCV for webcam integration
import speech_recognition as sr  # Speech recognition for voice commands
import aiohttp  # Asynchronous HTTP client
import asyncio  # For asynchronous operations
import gpiozero  # Library for controlling GPIO pins
import time  # Timing and delays
from aiohttp import web  # Web server for streaming

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"  # Replace with your ESP32's IP address

# Initialize servos for the robot's "ears"
left_servo = gpiozero.Servo(12)  # GPIO 12
right_servo = gpiozero.Servo(13)  # GPIO 13

# Function to control servo positions to indicate emotions or status
def move_servos(emotion):
    if emotion == "happy":
        left_servo.max()  # Move to max position
        right_servo.max()  # Move to max position
    elif emotion == "surprised":
        left_servo.mid()  # Move to mid position
        right_servo.mid()  # Move to mid position
    elif emotion == "angry":
        left_servo.min()  # Move to min position
        right_servo.min()  # Move to min position
    elif emotion == "wiggle":
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
                    try:
                        return await response.json()  # Return parsed JSON if possible
                    except Exception:
                        return await response.text()  # Return text if JSON parsing fails
                else:
                    print(f"GET request to {endpoint} failed with status {response.status}")
    except Exception as e:
        print(f"Error sending HTTP GET request: {e}")

# Asynchronous function to send HTTP POST requests to the ESP32
async def send_http_post(endpoint, data):
    try:
        async with aiohttp.ClientSession() as session:
            async with session.post(esp32_base_url + endpoint, json=data) as response:
                if response.status == 200:
                    print(f"POST request to {endpoint} succeeded")
                else:
                    print(f"POST request to {endpoint} failed with status {response.status}")
    except Exception as e:
        print(f"Error sending HTTP POST request: {e}")

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

# Asynchronous function to retrieve sensor data
async def get_sensor_data():
    sensor_data = await send_http_get("/sensors")
    if sensor_data:
        print("Sensor Data:", sensor_data)
    else:
        print("No sensor data retrieved")

# Asynchronous function for the figure-8 pattern with servo wiggles
async def figure_8_with_wiggles():
    # Define figure-8 pattern steps with servo wiggles
    await control_robot("forward")
    move_servos("wiggle")  # Add servo wiggles to the dance
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

# Asynchronous function for voice commands
async def handle_voice_commands():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)  # Adjust for background noise
        print("Listening for voice commands...")
        audio = recognizer.listen(source)  # Capture audio

    try:
        command = recognizer.recognize_google(audio)  # Recognize voice command
        print("Command received:", command)
        await control_robot(command.lower())  # Convert to lowercase to avoid case issues
    except sr.UnknownValueError:
        print("Could not understand the command")
    except sr.RequestError as e:
        print(f"Could not request results; {e}")

# Asynchronous function for video streaming with OpenCV
async def video_stream(request):
    # Using OpenCV to capture webcam feed
    cap = cv2.VideoCapture(0)  # 0 for the default webcam
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        _, img_encoded = cv2.imencode('.jpg', frame)  # Encode image
        return web.Response(body=img_encoded.tobytes(), content_type='image/jpeg')

# Main asynchronous function for robot control and additional tasks
async def main():
    print("Starting Pi control script for the ESP32-based robot")

    # Retrieve initial sensor data
    await get_sensor_data()

    # Perform a figure-8 dance with servo wiggles
    await figure_8_with_wiggles()

    # Handle voice commands
    await handle_voice_commands()

    # Set up the web server for video streaming
    app = web.Application()
    app.router.add_get('/video', video_stream)

    # Run the web server
    web.run_app(app, port=8080)

    # Ensure the robot stops at the end of the script
    await control_robot("stop")

    print("Pi control script execution complete.")

# Start the asynchronous event loop and run the main function
if __name__ == "__main__":
    asyncio.run(main())
