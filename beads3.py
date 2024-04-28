# Make sure to install necessary libraries
#!pip install numpy tensorflow aiohttp opencv-python
import asyncio
import aiohttp
import cv2
import random
import numpy as np
import tensorflow as tf
from collections import deque  # For replay memory
from tensorflow import keras  # Deep learning
from tensorflow.keras import layers  # Neural network layers

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

# Main asynchronous function
async def main():
    print("Starting reinforcement learning for the robot")

    current_state = await get_current_state()  # Get the initial state
    total_rewards = 0

    for episode in range(10):  # Simulate 10 episodes
        action = select_action(current_state)  # Select an action
        await control_robot(action)  # Execute the action
        await asyncio.sleep(1)  # Simulate time passing

        # Get the reward based on the action
        if action == "forward":
            reward = 10  # Reward for moving forward
        elif action == "stop":
            reward = 0  # Neutral reward for stopping
        else:
            reward = -10  # Penalty for moving backward or turning

        next_state = await get_current_state()  # Get the next state
        update_q_table(current_state, action, reward, next_state)  # Update Q-table

        total_rewards += reward
        current_state = next_state  # Move to the next state

    print(f"Total rewards: {total_rewards}")

# Start the event loop to run the main function
if __name__ == "__main__":
    asyncio.run(main())
