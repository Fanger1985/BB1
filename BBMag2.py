import asyncio
import aiohttp
import pigpio
import logging
import json
import heapq

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"

# Initialize pigpio for precise PWM servo control
pi = pigpio.pi()

# Mapping and Position Data
environment_map = {}
current_position = (0, 0)  # Simulated current position as (x, y) coordinates

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

async def move_robot(direction):
    command_url = esp32_base_url + direction
    response = await send_http_get(command_url)
    if response.get('status') == 'ok':
        update_position(direction)
        update_map(current_position, response.get('sensor_data'))
        logging.info(f"Robot moved {direction} to {current_position}")
        return True
    else:
        logging.info(f"Failed to move {direction}")
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
    environment_map[position] = data  # Assuming data is dict with sensor info

async def autonomous_exploration():
    # Autonomous navigation that updates the map and decides next moves
    while True:
        for direction in ["forward", "backward", "left", "right"]:
            success = await move_robot(direction)
            if success:
                break  # Move successful, break to process next cycle
        await asyncio.sleep(1)  # Throttle the request rate

async def main():
    global environment_map
    environment_map = load_map_from_file()  # Load the map at startup

    goal_position = (10, 10)  # Define goal position
    path = a_star(current_position, goal_position, environment_map)

    # Main operation loop
    while path:
        next_step = path.pop(0)
        direction_needed = determine_direction_to_move(current_position, next_step)
        await move_robot(direction_needed)
        if current_position == goal_position:
            logging.info("Reached goal position!")
            break  # Exit after reaching the goal
        # Recalculate path every few moves or after a significant event
        if not path or a_change_in_environment_detected():
            path = a_star(current_position, goal_position, environment_map)

        await asyncio.sleep(1)  # Simulate time delay for real-world operation

def cleanup():
    save_map_to_file()  # Save the map on cleanup
    logging.info("Cleaning up GPIO and servos...")
    pi.set_servo_pulsewidth(12, 0)  # Turn off servos
    pi.set_servo_pulsewidth(13, 0)
    pi.stop()
    logging.info("Cleanup complete. GPIO and servos are now safe.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cleanup()
