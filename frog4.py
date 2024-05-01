import aiohttp
import asyncio
import logging

# Setup basic configuration for logging
logging.basicConfig(level=logging.INFO)

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2"

async def fetch_sensor_data():
    url = f"{esp32_base_url}/sensors"
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status == 200:
                data = await response.json()
                logging.info(f"Sensor data received: {data}")
                return data
            else:
                logging.error(f"Failed to fetch sensor data, status code: {response.status}")
                return {}

async def send_command(command):
    url = f"{esp32_base_url}/{command}"
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status == 200:
                logging.info(f"Command '{command}' executed successfully.")
            else:
                logging.error(f"Failed to execute command '{command}', status code: {response.status}")

async def navigate_based_on_sensors_and_gyro():
    """Navigation logic based on sensor and gyro data."""
    sensor_data = await fetch_sensor_data()
    if not sensor_data:
        return

    distance = sensor_data.get('distance', 1000)  # Default to a safe distance if undefined
    ir_left = sensor_data.get('ir_left', 1)
    ir_right = sensor_data.get('ir_right', 1)
    ax = sensor_data.get('ax', 0)  # Acceleration x-axis
    ay = sensor_data.get('ay', 0)  # Acceleration y-axis
    az = sensor_data.get('az', 0)  # Acceleration z-axis
    gx = sensor_data.get('gx', 0)  # Gyro x-axis
    gy = sensor_data.get('gy', 0)  # Gyro y-axis
    gz = sensor_data.get('gz', 0)  # Gyro z-axis

    # Example gyro-based adjustments
    if abs(gx) > 10000 or abs(gy) > 10000 or abs(gz) > 10000:
        logging.info("Significant gyro movement detected, taking corrective action.")
        await send_command("stop")
        return

    # Sensor-based navigation logic:
    if distance < 30:
        await send_command("stop")
        await asyncio.sleep(1)  # Give some time to reassess situation
        if ir_left == 0:
            await send_command("right")
        elif ir_right == 0:
            await send_command("left")
        else:
            await send_command("backward")
    elif distance > 100:
        await send_command("forward")
    else:
        await send_command("stop")  # If in a moderate zone, stop and wait for further commands

async def emotional_response(emotion):
    """Handle emotional responses."""
    await send_command(f"expressEmotion?emotion={emotion}")

async def main():
    # Continuous loop to check sensor data and react accordingly
    try:
        while True:
            await navigate_based_on_sensors_and_gyro()
            await asyncio.sleep(0.1)  # Polling delay
    except KeyboardInterrupt:
        logging.info("Stopped by user.")

if __name__ == "__main__":
    asyncio.run(main())

