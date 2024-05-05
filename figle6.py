import aiohttp
import asyncio
import cv2
import numpy as np
import random
import logging
import threading
import json
import RPi.GPIO as GPIO
import time

# Setup basic logging
logging.basicConfig(filename='bb1_learning.log', level=logging.INFO, format='%(asctime)s:%(levelname)s:%(message)s')

# Configuration
esp32_base_url = "http://192.168.1.100/"
num_distance_states = 10
presence_states = 2
states = range(num_distance_states * presence_states)
actions = ['forward', 'backward', 'left', 'right', 'stop', 'return_home']
Q = np.zeros((len(states), len(actions)))
alpha = 0.1
gamma = 0.6
epsilon = 1.0
epsilon_decay = 0.995
epsilon_min = 0.1
home_base_location = None

# GPIO setup for servos
servo_left_pin = 12  # Left ear servo
servo_right_pin = 13  # Right ear servo
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_left_pin, GPIO.OUT)
GPIO.setup(servo_right_pin, GPIO.OUT)
left_servo = GPIO.PWM(servo_left_pin, 50)  # 50 Hz for servo
right_servo = GPIO.PWM(servo_right_pin, 50)
left_servo.start(0)
right_servo.start(0)

def save_q_table():
    """Saves the Q-table to a file."""
    with open("Q_table.json", "w") as f:
        json.dump(Q.tolist(), f)

def load_q_table():
    """Loads the Q-table from a file."""
    global Q
    try:
        with open("Q_table.json", "r") as f:
            Q = np.array(json.load(f))
    except FileNotFoundError:
        logging.info("No previous Q-table found, starting fresh.")

def update_q_table(state, action_index, reward, new_state):
    """Update the Q-value for a given state and action."""
    old_value = Q[state, action_index]
    future_optimal_value = np.max(Q[new_state])  # Best Q-value for the next state
    Q[state, action_index] = old_value + alpha * (reward + gamma * future_optimal_value - old_value)

def set_servo_angle(servo, angle):
    """Set servo to a specific angle."""
    duty = angle / 18 + 2
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def wiggle_servos():
    """Wiggles both servos to indicate activity or emotion using safe servo control."""
    angles = [30, 60, 90, 60, 30, 0, 30]  # Example angles for wiggling
    for angle in angles:
        safe_servo_control(left_servo, angle, 12)  # Left ear servo
        safe_servo_control(right_servo, angle, 13) # Right ear servo
        
def setup_initial_servo_positions():
    """Sets initial servo positions safely at startup."""
    safe_servo_control(left_servo, 0, 12)  # Set left ear servo to neutral position
    safe_servo_control(right_servo, 0, 13) # Set right ear servo to neutral position


def celebrate():
    """Special wiggle to celebrate completing a mission."""
    for _ in range(3):  # Fixed syntax here
        wiggle_servos()

async def send_http_get(session, endpoint):
    """Sends a GET request to the specified endpoint on the ESP32."""
    try:
        async with session.get(f"{esp32_base_url}{endpoint}") as response:
            if response.status == 200:
                return await response.json() if 'json' in response.headers.get('Content-Type', '') else await response.text()
            else:
                logging.error(f"Failed to execute {endpoint}: {response.status}")
                return None
    except Exception as e:
        logging.error(f"Error during HTTP GET to {endpoint}: {str(e)}")
        return None

async def get_state_from_sensors(session):
    """Retrieves sensor data from ESP32 and calculates the current state index."""
    response = await send_http_get(session, "sensors")
    if response:
        distance = response.get('distance', 100)
        ir_left = response.get('ir_left', 0)
        ir_right = response.get('ir_right', 0)

        distance_state = min(int(distance / 10), num_distance_states - 1)
        ir_state = 1 if ir_left > 0 or ir_right > 0 else 0
        state = distance_state + (num_distance_states * ir_state)
        return state
    return 0  # Return a default state if no data

def start_face_tracking():
    """Starts a separate thread for tracking faces using the webcam."""
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    cap = cv2.VideoCapture(0)

    def tracking():
        global home_base_location
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                faces = face_cascade.detectMultiScale(gray, 1.1, 4)
                if len(faces) > 0 and home_base_location is None:
                    home_base_location = {"x": faces[0][0], "y": faces[0][1]}
                    logging.info(f"Home base set at: {home_base_location}")
        except Exception as e:
            logging.error(f"Face tracking error: {str(e)}")
        finally:
            cap.release()

    threading.Thread(target=tracking, daemon=True).start()


async def navigate_to_home_base(session):
    """Navigates the robot back to the home base location using learned Q-values."""
    if home_base_location:
        current_state = await get_state_from_sensors(session)
        while current_state != home_base_location:
            action = actions[np.argmax(Q[current_state])]
            await send_http_get(session, action)
            current_state = await get_state_from_sensors(session)
            logging.info(f"Navigating back to home base, current state: {current_state}")
        logging.info("Returned to home base.")
    else:
        logging.error("Home base location is not set.")

async def enhanced_exploration(session):
    """Directs the robot to explore more aggressively towards open spaces."""
    while True:
        state = await get_state_from_sensors(session)
        if state < 5:  # Assuming state < 5 indicates more open space
            await send_http_get(session, 'forward')
        elif random.random() > 0.8:
            await send_http_get(session, random.choice(['left', 'right']))
        await asyncio.sleep(1)
        
def calculate_reward(state, new_state, action):
    # Customize this function based on BB1's goals and environment
    if new_state == goal_state:
        return 10  # Big reward for reaching a goal
    elif new_state == obstacle_state:
        return -10  # Big penalty for hitting an obstacle
    elif new_state != state:
        return 1  # Standard reward for successful movement
    else:
        return -1  # Penalty for no movement

async def robot_behavior(session):
    """Main behavior loop for the robot, applying Q-learning for decision making."""
    global epsilon
    load_q_table()  # Load the Q-table if it exists
    state = await get_state_from_sensors(session)  # Get initial state from sensors

    while True:
        # Epsilon-greedy strategy for action selection
        action_index = np.argmax(Q[state]) if random.random() > epsilon else random.randint(0, len(actions) - 1)
        action = actions[action_index]

        # Send the chosen action to the ESP32
        await send_http_get(session, action)

        # Observe the new state after taking the action
        new_state = await get_state_from_sensors(session)

        # Simulate receiving a reward based on the state transition
        reward = calculate_reward(state, new_state, action)
        update_q_table(state, action_index, reward, new_state)
        # Update the Q-table using the observed reward and the new state
        update_q_table(state, action_index, reward, new_state)

        # Update the current state
        state = new_state

        # Celebrate when returning to the home base
        if state == home_base_location:
            celebrate()

        # Randomly wiggle servos for liveliness
        if random.random() < 0.1:
            wiggle_servos()

        # Decay epsilon to reduce exploration over time
        epsilon = max(epsilon_min, epsilon * epsilon_decay)

        # Occasionally save the Q-table to disk
        if random.random() < 0.05:
            save_q_table()
            logging.info("Q-table saved.")

async def main():
    async with aiohttp.ClientSession() as session:
        threading.Thread(target=start_face_tracking).start()
        await asyncio.gather(robot_behavior(session), enhanced_exploration(session))

if __name__ == "__main__":
    asyncio.run(main())
