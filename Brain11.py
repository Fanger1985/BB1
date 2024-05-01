import asyncio
import aiohttp
import pigpio
import logging
import json
import os
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2"

# Initialize pigpio for precise PWM servo control
pi = pigpio.pi()

# Mapping and Position Data
environment_map = {}
current_position = (0, 0)

# Initialize Kalman Filter
kf = KalmanFilter(dim_x=2, dim_z=1)
kf.x = np.array([0., 0.])  # Initial state
kf.F = np.array([[1., 1.], [0., 1.]])  # State transition matrix
kf.H = np.array([[1., 0.]])  # Measurement function
kf.P *= 1000.  # Covariance matrix
kf.R = 5  # Measurement noise
kf.Q = Q_discrete_white_noise(dim=2, dt=1., var=0.1)  # Process noise

# Load TFLite model and allocate tensors
dir_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(dir_path, "mobilenet_v2_1.0_224.tflite")
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Initialize camera
camera = cv2.VideoCapture(0)  # Assuming a simple USB camera

def process_image(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (224, 224))
    return resized

async def send_http_get(endpoint):
    url = f"{esp32_base_url}{endpoint}"
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status == 200:
                return await response.json()
            else:
                logging.error(f"GET request to {url} failed with status {response.status}")

async def move_robot(command):
    response = await send_http_get(command)
    if response and response.get('status') == 'ok':
        logging.info(f"Command {command} executed successfully.")
        return True
    else:
        logging.error(f"Command {command} failed.")
        return False

async def get_sensor_data():
    sensor_data = await send_http_get("sensors")
    if sensor_data:
        logging.info(f"Sensor data: {sensor_data}")
        return sensor_data
    else:
        logging.error("No sensor data retrieved.")

async def figure_8():
    await move_robot("forward")
    await asyncio.sleep(1)
    await move_robot("stop")

    await move_robot("left")
    await asyncio.sleep(1)
    await move_robot("stop")

    await move_robot("forward")
    await asyncio.sleep(1)
    await move_robot("stop")

    await move_robot("right")
    await asyncio.sleep(1)
    await move_robot("stop")

async def main():
    logging.info("Starting control script for ESP32-based robot")
    await figure_8()  # Perform a figure-8 on startup

    # Continuous loop to fetch sensor data and perform autonomous exploration
    while True:
        sensor_data = await get_sensor_data()
        if sensor_data:
            # Implement AI and sensor-based decision making here
            await asyncio.sleep(1)  # Placeholder for real decision logic

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        if pi.connected:
            pi.stop()
        logging.info("Cleanup complete. GPIO and servos are now safe.")
