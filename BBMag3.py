import asyncio
import aiohttp
import pigpio
import logging
import json
import heapq
import cv2
import numpy as np
import tensorflow as tf
from tensorflow.keras import layers, models, Input, concatenate

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"

# Initialize pigpio for precise PWM servo control
pi = pigpio.pi()

# Mapping and Position Data
environment_map = {}
current_position = (0, 0)  # Simulated current position as (x, y) coordinates

def initialize_camera():
    # Initialize the camera
    camera = cv2.VideoCapture(0)  # Assuming a simple USB camera
    return camera

def capture_image(camera):
    try:
        ret, frame = camera.read()
        if ret:
            processed_image = process_image(frame)
            return processed_image
        else:
            logging.error("Failed to capture image from camera.")
            return None
    except Exception as e:
        logging.error(f"Error processing image from camera: {e}")
        return None

def process_image(frame):
    # Convert to grayscale and detect edges
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    return edges

def create_model(img_shape, num_sensors):
    """Create a multi-input model to process both image and sensor data."""
    # Image input branch
    img_input = Input(shape=img_shape)
    x = layers.Conv2D(32, (3, 3), activation='relu')(img_input)
    x = layers.MaxPooling2D((2, 2))(x)
    x = layers.Conv2D(64, (3, 3), activation='relu')(x)
    x = layers.Flatten()(x)

    # Sensor input branch
    sensor_input = Input(shape=(num_sensors,))
    y = layers.Dense(32, activation='relu')(sensor_input)

    # Combine branches
    combined = concatenate([x, y])
    z = layers.Dense(64, activation='relu')(combined)
    z = layers.Dense(4, activation='softmax')(z)

    model = models.Model(inputs=[img_input, sensor_input], outputs=z)
    model.compile(optimizer='adam', loss='sparse_categorical_crossentropy', metrics=['accuracy'])
    return model

model = create_model((160, 120, 1), num_sensors=5)  # Assuming 5 sensor readings

async def send_http_get(endpoint):
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(esp32_base_url + endpoint) as response:
                if response.status == 200:
                    logging.info(f"GET request to {endpoint} succeeded")
                    return await response.json()
                else:
                    logging.error(f"GET request to {endpoint} failed with status {response.status}")
    except Exception as e:
        logging.error(f"Error sending HTTP GET request: {e}")
    return {}

async def fetch_sensor_data_from_esp32():
    endpoint = "sensors"  # Placeholder endpoint
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(esp32_base_url + endpoint) as response:
                if response.status == 200:
                    data = await response.json()
                    return np.array(data['sensor_values'])  # Adjust as necessary
                else:
                    logging.error(f"Failed to fetch sensor data with status {response.status}")
                    return np.zeros(5)  # Return zero array if fetch fails
    except Exception as e:
        logging.error(f"Error fetching sensor data from ESP32: {e}")
        return np.zeros(5)  # Return zero array on exception

def get_local_sensor_data():
    # Placeholder for local sensor data collection
    local_sensor_data = np.random.rand(3)  # Random data simulating 3 local sensors
    return local_sensor_data

async def get_current_sensor_data():
    """
    Retrieve and combine sensor data from both the ESP32 and local Raspberry Pi sensors.
    """
    esp32_sensor_data = await fetch_sensor_data_from_esp32()
    local_sensor_data = get_local_sensor_data()
    combined_sensor_data = np.concatenate((esp32_sensor_data, local_sensor_data))
    return combined_sensor_data

async def move_robot(direction):
    direction_map = {0: 'forward', 1: 'backward', 2: 'left', 3: 'right'}
    command_url = esp32_base_url + direction_map[direction]
    try:
        response = await send_http_get(command_url)
        if response.get('status') == 'ok':
            update_position(direction_map[direction])
            update_map(current_position, response.get('sensor_data'))
            logging.info(f"Robot moved {direction_map[direction]} to {current_position}")
            return True
        else:
            logging.error(f"Failed to move {direction_map[direction]} with response {response}")
            return False
    except Exception as e:
        logging.error(f"Exception occurred while moving robot {direction_map[direction]}: {e}")
        return False


def update_position(direction):
    global current_position
    direction_to_offset = {
        "forward": (0, 1),
        "backward": (0, -1),
        "left": (-1, 0),
        "right": (1, 0)
    }
    offset = direction_to_offset.get(direction, (0, 0))
    current_position = (current_position[0] + offset[0], current_position[1] + offset[1])

def update_map(position, data):
    environment_map[position] = data  # Assuming data is a dict with sensor info

async def autonomous_exploration():
    camera = initialize_camera()
    try:
        while True:
            try:
                image = capture_image(camera)
                sensor_data = await get_current_sensor_data()
                if image is not None and sensor_data is not None:
                    image_input = np.expand_dims(image, axis=0) / 255.0
                    sensor_input = np.expand_dims(sensor_data, axis=0)
                    inputs = [image_input, sensor_input]
                    prediction = model.predict(inputs)
                    direction = np.argmax(prediction)
                    await move_robot(direction)
                await asyncio.sleep(1)
            except Exception as e:
                logging.error(f"Error during autonomous exploration loop: {e}")
    finally:
        camera.release()

async def main():
    try:
        global environment_map
        environment_map = load_map_from_file()  # Load the map at startup
        await autonomous_exploration()
    finally:
        cleanup()

def cleanup():
    try:
        save_map_to_file()  # Save the map on cleanup
        pi.set_servo_pulsewidth(12, 0)  # Turn off servos
        pi.set_servo_pulsewidth(13, 0)
        pi.stop()
        logging.info("Cleanup complete. GPIO and servos are now safe.")
    except Exception as e:
        logging.error(f"Error during cleanup: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cleanup()
