import aiohttp  # Asynchronous HTTP client
import asyncio  # For asynchronous operations
import time  # Timing and delays

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"  # Replace with your ESP32's IP address

# Asynchronous function to send HTTP GET requests to the ESP32
async def send_http_get(endpoint):
    async with aiohttp.ClientSession() as session:
        async with session.get(esp32_base_url + endpoint) as response:
            if response.status == 200:
                print(f"GET request to {endpoint} succeeded")
                try:
                    return await response.json()  # Return parsed JSON if possible
                except Exception:
                    return await response.text()  # Return text if JSON parsing fails
            else:
                print(f"GET request to {endpoint} failed with status {response.status}")

# Asynchronous function to send HTTP POST requests to the ESP32
async def send_http_post(endpoint, data):
    async with aiohttp.ClientSession() as session:
        async with session.post(esp32_base_url + endpoint, json=data) as response:
            if response.status == 200:
                print(f"POST request to {endpoint} succeeded")
            else:
                print(f"POST request to {endpoint} failed with status {response.status}")

# Asynchronous function to control the robot
async def control_robot(command):
    # Map commands to endpoint paths
    endpoints = {
        "forward": "/forward",
        "backward": "/backward",
        "left": "/left",
        "right": "/right",
        "stop": "/stop",
        "explore": "/explore",
        "dance": "/dance",
        "auto": "/auto",
    }

    # Ensure valid command
    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

# Asynchronous function to retrieve sensor data
async def get_sensor_data():
    sensor_data = await send_http_get("/sensors")
    if sensor_data:
        print("Sensor Data:", sensor_data)
    else:
        print("No sensor data retrieved")

# Asynchronous function to perform a figure-8 pattern
async def figure_8():
    # Define figure-8 pattern steps with delays
    await control_robot("forward")
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("left")
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("forward")
    await asyncio.sleep(1)
    await control_robot("stop")

    await control_robot("right")
    await asyncio.sleep(1)
    await control_robot("stop")

# Asynchronous function to handle AI operations or advanced logic
async def ai_operations():
    print("Placeholder for AI-related operations")

# Main asynchronous function
async def main():
    print("Starting Pi4 control script for the ESP32-based robot")

    # Retrieve initial sensor data
    await get_sensor_data()

    # Perform a figure-8 pattern
    await figure_8()

    # Execute AI-related operations (currently a placeholder)
    await ai_operations()

    # Ensure the robot stops at the end of the script
    await control_robot("stop")

    print("Pi4 control script execution complete.")

# Start the asynchronous event loop and run the main function
if __name__ == "__main__":
    asyncio.run(main())
