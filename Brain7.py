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

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

esp32_base_url = "http://192.168.1.2"

pi = pigpio.pi()

environment_map = {}
current_position = (0, 0)

def create_kalman_filter():
    kf = KalmanFilter(dim_x=2, dim_z=1)
    kf.x = np.array([0., 0.])
    kf.F = np.array([[1., 1.], [0., 1.]])
    kf.H = np.array([[1., 0.]])
    kf.P *= 1000.
    kf.R = 5
    kf.Q = Q_discrete_white_noise(dim=2, dt=1., var=0.1)
    return kf

kf = create_kalman_filter()

dir_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(dir_path, "mobilenet_v2_1.0_224.tflite")
interpreter = tflite.Interpreter(model_path=model_path)
interpreter.allocate_tensors()
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

def initialize_camera():
    camera = cv2.VideoCapture(0)
    return camera

def capture_image(camera):
    try:
        ret, frame = camera.read()
        if ret:
            return process_image(frame)
        else:
            logging.error("Failed to capture image from camera.")
            return None
    except Exception as e:
        logging.error(f"Error processing image from camera: {e}")
        return None

def process_image(frame):
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    resized = cv2.resize(rgb, (224, 224))
    return resized

async def send_http_get(endpoint):
    url = f"{esp32_base_url}/{endpoint}"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(url) as response:
                if response.status == 200:
                    return await response.json()
                else:
                    logging.error(f"GET request to {url} failed with status {response.status}")
    except Exception as e:
        logging.error(f"Error sending HTTP GET request to {url}: {e}")
    return {}

async def fetch_sensor_data_from_esp32():
    endpoint = "/sensors"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(esp32_base_url + endpoint) as response:
                if response.status == 200:
                    data = await response.json()
                    return np.array([data['ir_left'], data['ir_right'], data['distance']])
                else:
                    logging.error(f"Failed to fetch sensor data with status {response.status}")
                    return np.zeros(3)
    except Exception as e:
        logging.error(f"Exception when fetching sensor data: {e}")
        return np.zeros(3)

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
            measurement = response.get('sensor_data', {}).get('distance', 0)
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
    current_position = (current_position[0] + direction_to_offset[direction][0], current_position[1] + direction_to_offset[direction][1])

def update_map(position, data):
    environment_map[position] = data

async def autonomous_exploration():
    camera = initialize_camera()
    if not camera:
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
                    logging.error("Received incomplete sensor data")
            else:
                logging.error("Failed to capture image")
            await asyncio.sleep(1)
    except Exception as e:
        logging.error(f"Error during autonomous exploration loop: {e}")
    finally:
        if camera:
            camera.release()
            logging.info("Camera resource has been released.")

async def main():
    global environment_map
    environment_map = load_map_from_file()
    await autonomous_exploration()

def cleanup():
    if pi:
        if pi.connected:
            pi.set_servo_pulsewidth(12, 0)
            pi.set_servo_pulsewidth(13, 0)
            pi.stop()
        logging.info("Cleanup complete. GPIO and servos are now safe.")
    else:
        logging.info("Pigpio not connected.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cleanup()
