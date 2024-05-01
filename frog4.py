import aiohttp
import asyncio
import logging

# Setup logging
logging.basicConfig(level=logging.INFO)

# Robot's base URL
esp32_base_url = "http://192.168.1.2/"

# Environment map to track visited positions and observed obstacles
environment_map = {}

# Async HTTP request function
async def send_http_get(endpoint):
    url = f"{esp32_base_url}{endpoint}"
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status == 200:
                try:
                    return await response.json()
                except Exception:
                    return None
            else:
                logging.error(f"Failed to GET {url} with status {response.status}")
                return None

# Command control function for the robot
async def control_robot(command):
    await send_http_get(command)

# Fetch sensor and gyroscope data
async def get_robot_data():
    sensor_data = await send_http_get("sensors")
    gyro_data = await send_http_get("gyro")
    return sensor_data, gyro_data

# Obstacle reaction based on sensor data
async def react_to_obstacle(sensor_data):
    distance = sensor_data.get("distance", 100)
    if distance < 30:
        await control_robot("stop")
        await asyncio.sleep(1)
        await control_robot("left")  # Simplified reaction: always turn left
        await asyncio.sleep(1)
        await control_robot("stop")

# Update environment map with the current sensor data
def update_environment_map(current_position, sensor_data):
    environment_map[current_position] = sensor_data

# Monitor environment and respond to changes
async def monitor_environment():
    while True:
        sensor_data, gyro_data = await get_robot_data()
        current_position = gyro_data.get("position", {})  # Assuming position is tracked by gyro
        update_environment_map(current_position, sensor_data)
        await react_to_obstacle(sensor_data)
        await asyncio.sleep(1)

# Main routine
async def main():
    logging.info("Starting enhanced robot control script")
    # Task to monitor environment
    monitor_task = asyncio.create_task(monitor_environment())
    await asyncio.sleep(3600)  # Keep the script running for an hour
    monitor_task.cancel()  # Optionally stop the monitoring task

if __name__ == "__main__":
    asyncio.run(main())
