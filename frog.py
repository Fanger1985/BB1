import aiohttp  # Asynchronous HTTP client
import asyncio  # For asynchronous operations
import time  # Timing and delays

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2"  # Ensure correct IP address

async def send_http_get(endpoint):
    url = esp32_base_url + endpoint
    async with aiohttp.ClientSession() as session:
        async with session.get(url) as response:
            if response.status == 200:
                print(f"GET request to {url} succeeded")
                try:
                    return await response.json()
                except Exception:
                    return await response.text()
            else:
                print(f"GET request to {url} failed with status {response.status}")

async def control_robot(command):
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
    if command in endpoints:
        await send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

async def get_sensor_data():
    sensor_data = await send_http_get("/sensors")
    if sensor_data:
        print("Sensor Data:", sensor_data)
        return sensor_data
    else:
        print("No sensor data retrieved")
        return None

async def autonomous_navigation():
    sensor_data = await get_sensor_data()
    if sensor_data:
        distance = sensor_data.get("distance", 100)  # Default to safe distance if none reported
        if distance < 30:
            print("Obstacle too close! Stopping and reevaluating...")
            await control_robot("stop")
            await asyncio.sleep(1)
            await control_robot("left")  # Simple strategy: turn left when too close
            await asyncio.sleep(1)
            await control_robot("stop")
        else:
            print("Path clear. Moving forward.")
            await control_robot("forward")
    else:
        print("Failed to retrieve sensor data. Unable to navigate autonomously.")

async def main():
    print("Starting control script for the ESP32-based robot")
    await autonomous_navigation()  # Replace figure_8 with autonomous navigation
    print("Control script execution complete.")

if __name__ == "__main__":
    asyncio.run(main())
