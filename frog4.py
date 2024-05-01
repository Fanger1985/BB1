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

# Function to fetch and handle gyroscope and accelerometer data
async def fetch_motion_data():
    motion_data = await send_http_get("motion")
    if motion_data:
        ax, ay, az = motion_data['ax'], motion_data['ay'], motion_data['az']
        gx, gy, gz = motion_data['gx'], motion_data['gy'], motion_data['gz']
        # Implement motion processing logic here
        process_motion_data(ax, ay, az, gx, gy, gz)

def process_motion_data(ax, ay, az, gx, gy, gz):
    # Here you would implement checks for stability and sudden movements
    logging.info(f"Processing motion data: ax={ax}, ay={ay}, az={az}, gx={gx}, gy={gy}, gz={gz}")

# Command control function for the robot
async def control_robot(command):
    await send_http_get(command)

# Fetch sensor data and make decisions based on it
async def monitor_environment():
    while True:
        sensor_data = await send_http_get("sensors")
        process_sensor_data(sensor_data)
        await asyncio.sleep(1)

def process_sensor_data(data):
    if data:
        distance = data.get('distance')
        if distance < 30:  # For example, if too close to an obstacle
            asyncio.create_task(control_robot("stop"))
        else:
            asyncio.create_task(control_robot("forward"))
        # Update your environment map here
        update_environment_map(data)

def update_environment_map(data):
    position = data.get('position')  # Assuming position data is available
    environment_map[position] = data

async def main():
    logging.info("Starting enhanced robot control script")
    # Start monitoring tasks
    monitor_task = asyncio.create_task(monitor_environment())
    motion_task = asyncio.create_task(fetch_motion_data())
    await asyncio.sleep(3600)  # Run for 1 hour
    monitor_task.cancel()
    motion_task.cancel()

if __name__ == "__main__":
    asyncio.run(main())
