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
import keyboard  # Import the keyboard module to detect key presses

def save_map_to_file():
    try:
        with open('environment_map.json', 'w') as file:
            json.dump(environment_map, file)
        logging.info("Map saved successfully.")
    except Exception as e:
        logging.error(f"Failed to save map to file: {e}")

def load_map_from_file():
    try:
        with open('environment_map.json', 'r') as file:
            loaded_map = json.load(file)
        logging.info("Map loaded successfully.")
        return loaded_map
    except FileNotFoundError:
        logging.warning("No map file found. Starting with an empty map.")
        return {}
    except Exception as e:
        logging.error(f"Failed to load map from file: {e}")
        return {}

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2"

# Initialize pigpio for precise PWM servo control
pi = pigpio.pi()

# Mapping and Position Data
environment_map = {}
current_position = (0, 0)  # Simulated current position as (x, y) coordinates

# Initialize Kalman Filter
kf = create_kalman_filter()

# Load TFLite model and allocate tensors
dir_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(dir_path, "mobilenet_v2_1.0_224.tflite")
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

camera = initialize_camera()

async def figure_8():
    # Sequence of movements to create a figure-8 pattern
    await move_robot('forward')
    await asyncio.sleep(1)
    await move_robot('left')
    await asyncio.sleep(1)
    await move_robot('forward')
    await asyncio.sleep(1)
    await move_robot('right')
    await asyncio.sleep(1)
    await move_robot('stop')

async def keyboard_control():
    while True:
        await asyncio.sleep(0.1)  # Poll for keyboard input every 100 ms
        if keyboard.is_pressed('8'):  # If '8' key is pressed
            await figure_8()

async def main():
    # Run the keyboard listener alongside the main loop
    keyboard_task = asyncio.create_task(keyboard_control())
    try:
        while True:
            await fetch_sensor_data_from_esp32()
            await asyncio.sleep(1)  # Poll every second
    except asyncio.CancelledError:
        # Handle the cancellation properly
        pass
    finally:
        keyboard_task.cancel()
        await keyboard_task

if __name__ == "__main__":
    asyncio.run(main())
