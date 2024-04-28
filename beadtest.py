# First, ensure you have aiohttp installed
#!pip install aiohttpimport asyncio  # For asynchronous operations
import aiohttp  # Asynchronous HTTP client
import time

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"  # Replace with your ESP32's IP address

# Asynchronous function to send HTTP GET requests to the ESP32
async def send_http_get(endpoint):
    async with aiohttp.ClientSession() as session:
        async with session.get(esp32_base_url + endpoint) as response:
            if response.status == 200:
                print(f"Successfully sent GET request to {endpoint}")
                return await response.json()  # Return parsed JSON if available
            else:
                print(f"Error sending GET request to {endpoint}: {response.status}")

# Asynchronous function to send HTTP POST requests to the ESP32
async def send_http_post(endpoint, data):
    async with aiohttp.ClientSession() as session:
        async with session.post(esp32_base_url + endpoint, json=data) as response:
            if response.status == 200:
                print(f"Successfully sent POST request to {endpoint}")
            else:
                print(f"Error sending POST request to {endpoint}: {response.status}")

# Asynchronous function to control the robot
async def control_robot(command):
    endpoints = {
        "forward": "/forward",
        "backward": "/backward",
        "left": "/left",
        "right": "/right",
        "stop": "/stop",
        "explore": "/explore",
        "dance": "/dance",
        "auto": "/auto"
    }

    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

# Asynchronous function to retrieve sensor data
async def get_sensor_data():
    sensor_data = await send_http_get("/sensors")
    if sensor_data:
        print("Sensor Data:", sensor_data)

# Asynchronous function for AI-related operations
async def ai_operations():
    print("Placeholder for AI-based operations")
    # Future AI code or complex logic goes here

# Asynchronous function to perform a figure-8 pattern
async def figure_8():
    # Forward for 1 second
    await control_robot("forward")
    await asyncio.sleep(1)
    await control_robot("stop")
    
    # Spin left for 1 second
    await control_robot("left")
    await asyncio.sleep(1)
    await control_robot("stop")
    
    # Forward for 1 second
    await control_robot("forward")
    await asyncio.sleep(1)
    await control_robot("stop")
    
    # Spin right for 1 second
    await control_robot("right")
    await asyncio.sleep(1)
    await control_robot("stop")

# Main asynchronous function
async def main():
    print("Starting Pi4 control script for the ESP32-based robot")

    # Retrieve initial sensor data
    await get_sensor_data()
    
    # Perform the figure-8 pattern
    await figure_8()

    # Placeholder for AI-related operations
    await ai_operations()

    print("Pi4 control script execution complete.")

# Start the event loop and run the main function
if __name__ == "__main__":
    asyncio.run(main())

