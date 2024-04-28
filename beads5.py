import asyncio
import aiohttp
import RPi.GPIO as GPIO
import cv2
import random
import numpy as np
import tensorflow as tf
from collections import deque
from tensorflow import keras
from tensorflow.keras import layers
import time

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.4.1/"  # Adjust to your ESP32's IP address

# Define actions and states
actions = ["forward", "backward", "left", "right", "stop"]
state_space_size = 5  # Adjust based on the number of sensors and observations

# Initialize Q-table for Q-Learning
q_table = np.zeros((state_space_size, len(actions)))

# Hyperparameters
learning_rate = 0.1
discount_factor = 0.99
epsilon = 0.1  # Exploration-exploitation trade-off

# Replay memory for DQNs
replay_memory = deque(maxlen=10000)

# Servo setup on Raspberry Pi GPIO pins 12 and 13
GPIO.setmode(GPIO.BCM)
servo_pin1 = 12
servo_pin2 = 13
GPIO.setup(servo_pin1, GPIO.OUT)
GPIO.setup(servo_pin2, GPIO.OUT)
servo1 = GPIO.PWM(servo_pin1, 50)  # 50 Hz
servo2 = GPIO.PWM(servo_pin2, 50)  # 50 Hz
servo1.start(7.5)  # Neutral position
servo2.start(7.5)  # Neutral position

# Function to wiggle the servos (for personality)
def wiggle_servos():
    for angle in range(70, 110, 10):
        servo1.ChangeDutyCycle(2.5 + angle / 18)
        servo2.ChangeDutyCycle(2.5 + (180 - angle) / 18)
        time.sleep(0.1)
    for angle in range(110, 70, -10):
        servo1.ChangeDutyCycle(2.5 + angle / 18)
        servo2.ChangeDutyCycle(2.5 + (180 - angle) / 18)
        time.sleep(0.1)

# Asynchronous function to send HTTP GET requests
async def send_http_get(endpoint):
    async with aiohttp.ClientSession() as session:
        async with session.get(esp32_base_url + endpoint) as response:
            if response.status == 200:
                return await response.json()
            else:
                print(f"Error sending GET request to {endpoint}: {response.status}")
                return None

# Asynchronous function to control the robot's movement
async def control_robot(action):
    await send_http_get(f"/{action}")
    if action in ["forward", "left", "right"]:
        wiggle_servos()  # Wiggle servos for personality

# Function to get the current state from sensors
async def get_current_state():
    sensor_data = await send_http_get("/sensors")
    if sensor_data:
        # Convert sensor data into a state representation
        state = [
            sensor_data.get("ir_left", 0),
            sensor_data.get("ir_right", 0),
            sensor_data.get("distance", 0),
            random.randint(0, 100),  # Random state for exploration
        ]
        return np.array(state)
    return np.zeros(state_space_size)

# Function to select an action based on epsilon-greedy policy
def select_action(state):
    if random.uniform(0, 1) < epsilon:
        return random.choice(actions)  # Explore
    else:
        return actions[np.argmax(q_table[state])]  # Exploit

# Function to update the Q-table for Q-Learning
def update_q_table(state, action, reward, next_state):
    action_idx = actions.index(action)
    best_next_action = np.argmax(q_table[next_state])
    td_target = reward + discount_factor * q_table[next_state][best_next_action]
    td_error = td_target - q_table[state][action_idx]
    q_table[state][action_idx] += learning_rate * td_error

# Asynchronous function to "run away"
async def run_away():
    await control_robot("backward")
    await asyncio.sleep(1)
    if random.choice([True, False]):
        await control_robot("left")
    else:
        await control_robot("right")
    await asyncio.sleep(1)
    await control_robot("stop")

# Asynchronous function to "follow"
async def follow():
    await control_robot("forward")
    await asyncio.sleep(1)
    await control_robot("stop")

# Main asynchronous function
async def main():
    print("Starting reinforcement learning for the robot")

    current_state = await get_current_state()  # Get the initial state
    total_rewards = 0

    for episode in range 10):  # Simulate 10 episodes
        action = select_action(current_state)  # Select an action
        await control_robot(action)  # Execute the action
        await asyncio.sleep(1)  # Simulate time passing

        # Reward logic based on actions
        if action == "forward":
            reward = 10  # Reward for moving forward
        elif action == "stop":
            reward is neutral
        else:
            reward = -10  # Penalty for other movements

        next_state = await get_current_state()  # Get the next state
        update_q_table(current_state, action, reward, next_state)  # Update Q-table

        total_rewards += reward
        current_state = next_state  # Move to the next state

    print(f"Total rewards: {total_rewards}")

    # Cleanup and stop servos
    servo1.stop()
    servo2.stop()
    GPIO.cleanup()

# Start the event loop to run the main function
if __name__ == "__main__":
    asyncio.run(main())
