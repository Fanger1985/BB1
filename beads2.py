import asyncio
import aiohttp
import math
import time

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.4.1"  # Adjust to your ESP32's IP address

# Thresholds for gyroscope detection
GYRO_THRESHOLD = 15000  # Tilt threshold to detect lifting/placement

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

# Asynchronous function to retrieve sensor data, including gyroscope
async def get_sensor_data():
    data = await send_http_get("/sensors")
    return data

# Asynchronous function to monitor the gyroscope for lifting/placement detection
async def monitor_gyroscope():
    while True:
        sensor_data = await get_sensor_data()
        if sensor_data:
            ax = sensor_data.get("ax")
            ay = sensor_data.get("ay")
            az = sensor_data.get("az")

            if any(abs(val) > GYRO_THRESHOLD for val in (ax, ay, az)):
                print("Robot is lifted. Entering chill mode...")
                await control_robot("stop")  # Enter chilling state
                while any(abs(val) > GYRO_THRESHOLD for val in (ax, ay, az)):  # Keep chilling
                    await asyncio.sleep(1)  # Recheck every second
                print("Robot placed down. Checking for room change...")
                break  # Exit loop once placed down

        await asyncio.sleep(1)  # Check every second

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

# Main asynchronous function to execute the mission
async def main():
    print("Starting Pi4 control script for the ESP32-based robot")

    # Monitor the gyroscope for lifting/placement
    await monitor_gyroscope()

    # Perform room mapping
    room_map = await room_mapping()

    # Calculate the center of the room
    room_center = calculate_room_center(room_map)

    # Move to the room's center
    await move_to_center(room_center)

    print("Mission complete.")

# Start the event loop to run the main function
if __name__ == "__main__":
    asyncio.run(main())
