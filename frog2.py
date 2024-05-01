import aiohttp
import asyncio
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2"

async def send_http_get(endpoint):
    url = esp32_base_url + endpoint
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status == 200:
                logging.info(f"GET request to {url} succeeded")
                try:
                    return await response.json()
                except Exception:
                    return await response.text()
            else:
                logging.error(f"GET request to {url} failed with status {response.status}")
                return None

async def control_robot(command):
    endpoints = {
        "forward": "forward",
        "backward": "backward",
        "left": "left",
        "right": "right",
        "stop": "stop",
        "explore": "explore",
        "dance": "dance",
        "auto": "auto",
    }
    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        logging.error(f"Invalid command: {command}")

async def get_sensor_data():
    sensor_data = await send_http_get("sensors")
    if sensor_data:
        logging.info(f"Sensor Data: {sensor_data}")
        return sensor_data
    else:
        logging.error("Failed to retrieve sensor data")
        return {}

async def smart_navigation():
    while True:
        sensor_data = await get_sensor_data()
        if sensor_data:
            distance = sensor_data.get("distance", 100)  # Default if no distance is reported
            ir_left = sensor_data.get("ir_left", 1)
            ir_right = sensor_data.get("ir_right", 1)
            
            if distance < 30 or ir_left == 0 or ir_right == 0:
                logging.info("Obstacle detected! Stopping and deciding next move...")
                await control_robot("stop")
                if ir_left == 0:
                    await control_robot("right")
                else:
                    await control_robot("left")
                await asyncio.sleep(1)  # Pause for a moment after turning
                await control_robot("stop")  # Ensure it stops before next action
            else:
                logging.info("Path clear. Moving forward.")
                await control_robot("forward")
        await asyncio.sleep(0.5)  # Check sensor data at a reasonable interval

async def main():
    logging.info("Starting the smart navigation control script for the ESP32-based robot")
    await smart_navigation()

if __name__ == "__main__":
    asyncio.run(main())
