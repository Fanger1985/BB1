import asyncio
import aiohttp
import pigpio
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"

# Initialize pigpio for precise PWM servo control
pi = pigpio.pi()

# Servo GPIO pins
servo1_pin = 12
servo2_pin = 13

# Servo control parameters
angle_0 = 500  # Pulse width for 0 degrees
angle_180 = 2500  # Pulse width for 180 degrees
current_angle1 = angle_0
current_angle2 = angle_0

def smooth_move_servos(pin, start_angle, end_angle, step=10):
    # Gradually move the servo to a new position
    range_func = range if end_angle > start_angle else lambda start, end, step: range(start, end, -step)
    for angle in range_func(start_angle, end_angle, step):
        pi.set_servo_pulsewidth(pin, angle)
        asyncio.sleep(0.02)  # Small delay for smoother transitions

async def send_http_get(endpoint):
    # Send HTTP GET requests safely with aiohttp
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(esp32_base_url + endpoint) as response:
                if response.status == 200:
                    logging.info(f"GET request to {endpoint} succeeded")
                else:
                    logging.error(f"GET request to {endpoint} failed with status {response.status}")
    except Exception as e:
        logging.error(f"Error sending HTTP GET request: {e}")

async def control_robot(command):
    # Control robot by sending HTTP GET commands to the ESP32 server
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
        logging.error(f"Invalid command: {command}")

async def fetch_sensor_data():
    sensor_url = esp32_base_url + "sensors"
    try:
        async with aiohttp.ClientSession() as session:
            async with session.get(sensor_url) as response:
                if response.status == 200:
                    data = await response.json()
                    return data
                else:
                    logging.error(f"Failed to fetch sensor data: HTTP {response.status}")
    except Exception as e:
        logging.error(f"Error fetching sensor data: {e}")
    return {}

async def autonomous_decisions():
    data = await fetch_sensor_data()
    if data:
        distance = data.get("distance")
        if distance and distance < 30:  # Assuming 30 cm is too close to an obstacle
            await control_robot("stop")
            await asyncio.sleep(1)
            await control_robot("left")  # Make a left turn to avoid the obstacle
            await asyncio.sleep(1)
            await control_robot("forward")  # Continue moving forward after the turn
            logging.info("Obstacle avoided: Executed left turn")

async def main():
    # Running autonomous decisions alongside regular tasks
    await asyncio.gather(
        autonomous_decisions(),
        figure_8_with_wiggles()
    )

def cleanup():
    # Clean up GPIO and servo control
    logging.info("Cleaning up GPIO and servos...")
    pi.set_servo_pulsewidth(servo1_pin, 0)  # Turn off servo 1
    pi.set_servo_pulsewidth(servo2_pin, 0)  # Turn off servo 2
    pi.stop()
    logging.info("Cleanup complete. GPIO and servos are now safe.")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    finally:
        cleanup()
