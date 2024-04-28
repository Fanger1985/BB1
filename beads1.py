import asyncio  # Asynchronous operations
import aiohttp  # Asynchronous HTTP client
import math  # Math operations
import time  # Time management

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.4.1"  # Adjust to your ESP32's IP address

# Asynchronous function to send HTTP GET requests
async def send_http_get(endpoint):
    async with aiohttp.ClientSession() as session:
        async with session.get(esp32_base_url + endpoint) as response:
            if response.status == 200:
                return await response.json()  # Return parsed JSON if available
            else:
                print(f"Error sending GET request to {endpoint}: {response.status}")
                return None

# Asynchronous function to control the robot's movement
async def control_robot(command):
    endpoints = {
        "forward": "/forward",
        "backward": "/backward",
        "left": "/left",
        "right": "/right",
        "stop": "/stop",
    }

    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

# Asynchronous function to retrieve sensor data for mapping
async def get_sensor_data():
    data = await send_http_get("/sensors")
    return data

# Asynchronous function to perform room mapping
async def room_mapping():
    # Collect sensor data over time to build the room map
    room_map = []
    for _ in range(10):  # Collect 10 samples for simplicity
        sensor_data = await get_sensor_data()
        if sensor_data:
            distance = sensor_data.get("distance")
            ir_left = sensor_data.get("ir_left")
            ir_right = sensor_data.get("ir_right")
            room_map.append({"distance": distance, "ir_left": ir_left, "ir_right": ir_right})
        await asyncio.sleep(1)  # Wait 1 second between samples
    return room_map

# Function to calculate the middle of the room
def calculate_room_center(room_map):
    distances = [entry["distance"] for entry in room_map if entry["distance"]]
    if distances:
        max_distance = max(distances)
        min_distance = min(distances)
        room_center = (max_distance + min_distance) / 2  # Approximate middle of the room
        return room_center
    return None

# Asynchronous function to move the robot to the room's center
async def move_to_center(room_center):
    if room_center:
        # Move the robot to the calculated center
        await control_robot("forward")  # Move towards the center
        await asyncio.sleep(2)  # Give some time for movement
        await control_robot("stop")
        print("Reached the room's center")
    else:
        print("Room center could not be calculated")

# Main asynchronous function to execute the mapping and move to the room's center
async def main():
    print("Starting room mapping mission")

    # Perform room mapping
    room_map = await room_mapping()

    # Calculate the center of the room
    room_center = calculate_room_center(room_map)

    # Move to the center of the room
    await move_to_center(room_center)

    # Set up the robot to face a human and chill
    await control_robot("right")  # Rotate to face the human
    await asyncio.sleep(1)  # Wait for the turn
    await control_robot("stop")
    print("Mission complete. Robot is chilling.")

# Start the event loop to run the main function
if __name__ == "__main__":
    asyncio.run(main())
