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
def create_kalman_filter():
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.x = np.array([0., 0.])  # Initial state (location and velocity)
    kf.F = np.array([[1., 1.],  # State transition matrix
                     [0., 1.]])
    kf.H = np.array([[1., 0.]])  # Measurement function
    kf.P *= 1000.                 # Covariance matrix
    kf.R = 5                      # Measurement noise
    kf.Q = Q_discrete_white_noise(dim=2, dt=1., var=0.1)  # Process noise
    return kf

kf = create_kalman_filter()

# Load TFLite model and allocate tensors
dir_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(dir_path, "mobilenet_v2_1.0_224.tflite")
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

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
    # Convert to RGB and resize for MobileNetV2
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (224, 224))
    return resized

async def send_http_get(endpoint):
    url = f"{esp32_base_url}/{endpoint}"  # Make sure the URL is properly formed
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                if response.status == 200:
                    logging.info(f"GET request to {url} succeeded")
                    return await response.json()
                else:
                    logging.error(f"GET request to {url} failed with status {response.status}")
    except Exception as e:
        logging.error(f"Error sending HTTP GET request to {url}: {e}")
    return {}

async def fetch_sensor_data_from_esp32():
    endpoint = "/sensors"
    url = esp32_base_url + endpoint
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                if response.status == 200:
                    data = await response.json()
                    logging.info(f"Sensor data received: {data}")
                    handle_sensor_data(data)
                else:
                    logging.error(f"Failed to fetch sensor data with status {response.status}")
    except Exception as e:
        logging.error(f"Exception when fetching sensor data: {e}")
        
def handle_sensor_data(data):
    ir_left = data.get('ir_left')
    ir_right = data.get('ir_right')
    distance = data.get('distance')
    logging.info(f"IR Left: {ir_left}, IR Right: {ir_right}, Distance: {distance}")
    
    if distance < 20:
        logging.info("Obstacle very close. Taking evasive action.")
        # Insert logic to handle close obstacles, e.g., stop or change direction
    elif ir_left == 0 or ir_right == 0:
        logging.info("Edge detected. Adjusting path.")
       


async def get_current_sensor_data():
    esp32_sensor_data = await fetch_sensor_data_from_esp32()
    local_sensor_data = np.random.rand(3)  # Simulate local sensor data
    combined_sensor_data = np.concatenate((esp32_sensor_data, local_sensor_data))
    return combined_sensor_data

async def move_robot(direction):
    direction_map = {0: 'forward', 1: 'backward', 2: 'left', 3: 'right'}
    command_url = esp32_base_url + '/' + direction_map[direction]
    try:
        response = await send_http_get(command_url)
        if response.get('status') == 'ok':
            sensor_data = response.get('sensor_data', {})
            measurement = sensor_data.get('distance', 0)
            kf.predict()
            kf.update(np.array([measurement]))
            update_position(direction_map[direction], kf.x[0])
            update_map(current_position, response.get('sensor_data'))
            logging.info(f"Robot moved {direction_map[direction]} to {current_position}")
            return True
        else:
            logging.error(f"Failed to move {direction_map[direction]} with response {response}")
            return False
    except Exception as e:
        logging.error(f"Exception occurred while moving robot {direction_map[direction]}: {e}")
        return False

def update_position(direction, estimated_position):
    global current_position
    direction_to_offset = {"forward": (0, 1), "backward": (0, -1), "left": (-1, 0), "right": (1, 0)}
    offset = direction_to_offset[direction]
    current_position = (current_position[0] + offset[0], current_position[1] + offset[1])

def update_map(position, data):
    environment_map[position] = data

async def autonomous_exploration():
    camera = initialize_camera()
    if camera is None:
        logging.error("Failed to initialize camera.")
        return
    try:
        while True:
            image = capture_image(camera)
            if image is not None:
                sensor_data = await get_current_sensor_data()
                if sensor_data.size == 3:
                    image_input = np.expand_dims(image, axis=0).astype(np.float32)
                    sensor_input = np.expand_dims(sensor_data, axis=0).astype(np.float32)
                    interpreter.set_tensor(input_details[0]['index'], image_input)
                    interpreter.set_tensor(input_details[1]['index'], sensor_input)
                    interpreter.invoke()
                    predictions = interpreter.get_tensor(output_details[0]['index'])
                    if predictions.size > 0:
                        direction = np.argmax(predictions)
                        await move_robot(direction)
                    else:
                        logging.error("No predictions from model")
                else:
                    logging.error(f"Received incomplete sensor data: {sensor_data}")
            else:
                logging.error("Failed to capture image.")
            await asyncio.sleep(1)
    except Exception as e:
        logging.error(f"Error during autonomous exploration loop: {e}")
    finally:
        if camera:
            camera.release()
            logging.info("Camera resource has been released.")

async def main():
    while True:
        await fetch_sensor_data_from_esp32()
        await asyncio.sleep(1)  # Poll every second

def cleanup():
    try:
        if pi is not None:
            if pi.connected:
                pi.set_servo_pulsewidth(12, 0)
                pi.set_servo_pulsewidth(13, 0)
                pi.stop()
            else:
                logging.info("Pigpio not connected.")
        logging.info("Cleanup complete. GPIO and servos are now safe.")
    except Exception as e:
        logging.error(f"Error during cleanup: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cleanup()

