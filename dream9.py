import asyncio
import aiohttp
import random
import logging
from collections import deque
import RPi.GPIO as GPIO
import time
import board
import busio
from adafruit_vl53l4cd import VL53L4CD
import numpy as np
import json
from adafruit_apds9960.apds9960 import APDS9960
import heapq

# Setup logging
logging.basicConfig(level=logging.DEBUG)

# Constants for the ESP32's IP and the endpoint URIs
ESP32_IP = '192.168.4.1'  # ESP32 IP address in AP mode
BASE_URL = f'http://{ESP32_IP}'

# Q-learning parameters
ALPHA = 0.1  # Learning rate
GAMMA = 0.9  # Discount factor
EPSILON = 0.1  # Exploration rate

# Experience replay buffer
REPLAY_BUFFER = deque(maxlen=2000)

# Map grid and robot position
MAP_GRID = [[0 for _ in range(10)] for _ in range(10)]
CURRENT_POSITION = [5, 5]
GOAL_POSITION = [7, 6]  # Example goal position

# Initialize Q-table
Q_TABLE = np.zeros((10, 10, 5))  # 10x10 grid with 5 possible actions (forward, backward, spinLeft, spinRight, stop)
ACTIONS = ['/forward', '/backward', '/spinLeft', '/spinRight', '/stop']

# GPIO setup
GPIO.setmode(GPIO.BCM)
LEFT_EAR_PIN = 12
RIGHT_EAR_PIN = 13

GPIO.setup(LEFT_EAR_PIN, GPIO.OUT)
GPIO.setup(RIGHT_EAR_PIN, GPIO.OUT)

left_ear = GPIO.PWM(LEFT_EAR_PIN, 50)  # 50Hz frequency
right_ear = GPIO.PWM(RIGHT_EAR_PIN, 50)  # 50Hz frequency

left_ear.start(0)
right_ear.start(0)

# ToF sensor setup
i2c = busio.I2C(board.SCL, board.SDA)
tof_sensor = VL53L4CD(i2c)
tof_sensor.distance_mode = 2  # Long range mode
tof_sensor.start_ranging()

# Proximity/Light/RGB sensor setup
apds = APDS9960(i2c)
apds.enable_proximity = True
apds.enable_color = True
apds.enable_gesture = True

# Battery monitoring
BATTERY_PIN = 4
GPIO.setup(BATTERY_PIN, GPIO.IN)

class Emotion:
    def __init__(self):
        self.mood_points = 0

    def update_mood(self, points):
        self.mood_points += points

    def check_mood(self):
        if self.mood_points > 50:
            return "happy"
        elif self.mood_points > 0:
            return "curious"
        elif self.mood_points < -50:
            return "scared"
        else:
            return "neutral"

def initialize_sensors():
    logging.debug("Initializing sensors...")
    logging.debug("Sensors initialized.")

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, grid):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))

    while oheap:
        current = heapq.heappop(oheap)[1]

        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < len(grid):
                if 0 <= neighbor[1] < len(grid[0]):
                    if grid[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue

            if tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap]:
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))

    return False

def decide_next_move(mood, position, map, front_dist, rear_dist, tof_dist, prox_dist, gyro_data):
    if prox_dist < 10:  # Immediate obstacle close to head
        logging.debug("Obstacle detected near head! Moving backward.")
        return '/backward'
    elif mood == "happy" and front_dist > 1000 and tof_dist > 1000:
        return '/forward'
    elif mood == "curious" and front_dist > 500:
        return '/spinRight'
    elif mood == "scared" or front_dist < 200 or rear_dist < 200 or tof_dist < 200:
        return '/backward'
    elif abs(gyro_data['gx']) > 15000 or abs(gyro_data['gy']) > 15000 or abs(gyro_data['gz']) > 15000:
        return '/stop'
    elif mood == "neutral":
        return patrol_route(position)
    else:
        return random.choice(['/spinLeft', '/spinRight', '/forward', '/stop'])

def patrol_route(position):
    x, y = position
    if y < 9:
        return '/forward'
    elif x < 9:
        return '/spinRight'
    elif y > 0:
        return '/backward'
    elif x > 0:
        return '/spinLeft'
    return '/forward'

def update_position(command):
    global CURRENT_POSITION
    x, y = CURRENT_POSITION
    if command == '/forward':
        y += 1
    elif command == '/backward':
        y -= 1
    elif command == '/spinRight':
        x += 1
    elif command == '/spinLeft':
        x -= 1
    x = max(0, min(x, len(MAP_GRID[0]) - 1))
    y = max(0, min(y, len(MAP_GRID) - 1))
    CURRENT_POSITION = [x, y]

def choose_action(state):
    if random.uniform(0, 1) < EPSILON:
        return random.choice(ACTIONS)
    else:
        state_action = Q_TABLE[state[0], state[1], :]
        return ACTIONS[np.argmax(state_action)]

def learn(state, action, reward, next_state):
    action_index = ACTIONS.index(action)
    predict = Q_TABLE[state[0], state[1], action_index]
    target = reward + GAMMA * np.max(Q_TABLE[next_state[0], next_state[1], :])
    Q_TABLE[state[0], state[1], action_index] += ALPHA * (target - predict)

def get_state():
    x, y = CURRENT_POSITION
    return (x, y)

def get_reward(proximity, front_distance, rear_distance, tof_distance):
    if proximity < 10 or front_distance < 200 or tof_distance < 200:
        return -100
    if proximity > 1000 or front_distance > 1000 or tof_distance > 1000:
        return 50
    return -1

def replay_experience():
    global REPLAY_BUFFER
    batch_size = min(32, len(REPLAY_BUFFER))
    minibatch = random.sample(REPLAY_BUFFER, batch_size)
    for state, action, reward, next_state in minibatch:
        learn(state, action, reward, next_state)

async def fetch_with_retries(session, url, max_retries=5, timeout=10):
    for attempt in range(max_retries):
        try:
            async with session.get(url, timeout=timeout) as response:
                if response.status == 200:
                    return await response.json()
                else:
                    logging.error(f'Failed to fetch {url}: Status {response.status}')
        except aiohttp.ClientError as e:
            logging.error(f'Error fetching {url} on attempt {attempt + 1}: {e}')
            await asyncio.sleep(2)  # Wait before retrying
    return None

async def get_sensor_data(session):
    while True:
        try:
            sensor_data = await fetch_with_retries(session, f"{BASE_URL}/sensors")
            gyro_data = await fetch_with_retries(session, f"{BASE_URL}/gyro")

            if sensor_data and gyro_data:
                await make_decision_based_on_sensors(sensor_data, gyro_data)

            await asyncio.sleep(1)  # Add delay to avoid overloading
        except Exception as e:
            logging.error(f'Error in get_sensor_data: {e}')
            await asyncio.sleep(1)  # Add delay before retrying

async def make_decision_based_on_sensors(sensor_data, gyro_data):
    try:
        logging.debug(f"Sensor Data: {sensor_data}, Gyro Data: {gyro_data}")
        front_distance = sensor_data.get('front_distance', 1000)
        rear_distance = sensor_data.get('rear_distance', 1000)
        proximity = min(front_distance, rear_distance)
        tof_distance = tof_sensor.distance
        proximity_sensor = apds.proximity

        logging.debug(f"ToF Distance: {tof_distance}")
        logging.debug(f"Proximity Sensor Distance: {proximity_sensor}")
        logging.debug(f"Gyro Data: {gyro_data}")

        mood = emotion_system.check_mood()
        state = get_state()
        action = decide_next_move(mood, state, MAP_GRID, front_distance, rear_distance, tof_distance, proximity_sensor, gyro_data)

        set_ears_based_on_action(action)

        await send_command(action)
        next_state = get_state()
        reward = get_reward(proximity, front_distance, rear_distance, tof_distance)
        REPLAY_BUFFER.append((state, action, reward, next_state))
        replay_experience()
    except Exception as e:
        logging.error(f'Error in make_decision_based_on_sensors: {e}')

async def send_command(endpoint):
    try:
        update_position(endpoint)
        async with aiohttp.ClientSession() as session:
            async with session.get(f"{BASE_URL}{endpoint}", timeout=5) as response:
                if response.status == 200:
                    logging.debug(f'Command {endpoint} executed successfully!')
                else:
                    logging.error(f'Failed to execute {endpoint}: Status {response.status}')
    except Exception as e:
        logging.error(f'Error sending command {endpoint}: {e}')

async def dynamic_navigation():
    global emotion_system
    emotion_system = Emotion()
    async with aiohttp.ClientSession() as session:
        await get_sensor_data(session)

def move_ears(left_angle, right_angle):
    left_duty = (left_angle / 18) + 2
    right_duty = (right_angle / 18) + 2
    left_ear.ChangeDutyCycle(left_duty)
    right_ear.ChangeDutyCycle(right_duty)
    time.sleep(0.5)

def set_ears_based_on_action(action):
    if action == "/forward":
        move_ears(160, 20)  # Left ear back, Right ear forward
    elif action in ["/spinLeft", "/spinRight"]:
        move_ears(90, 90)  # Ears in neutral position
    elif action == "/backward":
        move_ears(45, 135)  # Ears alert

async def main():
    initialize_sensors()
    asyncio.create_task(dynamic_navigation())

    while True:
        await asyncio.sleep(0.1)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        left_ear.stop()
        right_ear.stop()
        GPIO.cleanup()
