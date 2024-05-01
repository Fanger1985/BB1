import aiohttp
import asyncio
import logging
import random

logging.basicConfig(level=logging.INFO)

esp32_base_url = "http://192.168.1.2/"

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

async def control_robot(command):
    endpoint = {
        "forward": "forward",
        "backward": "backward",
        "left": "left",
        "right": "right",
        "stop": "stop",
        "dance": "dance",
        "expressEmotion": "expressEmotion",
        "auto": "auto"
    }.get(command, "")
    if endpoint:
        await send_http_get(endpoint)

async def get_sensor_data():
    data = await send_http_get("sensors")
    return data if data else {}

async def react_to_obstacle(distance, ir_left, ir_right):
    if distance < 30 or ir_left == 0 or ir_right == 0:
        await control_robot("stop")
        await asyncio.sleep(1)
        if ir_left == 0:
            await control_robot("right")
        else:
            await control_robot("left")
        await asyncio.sleep(1)
        await control_robot("stop")

async def monitor_environment():
    while True:
        sensor_data = await get_sensor_data()
        distance = sensor_data.get("distance", 100)
        ir_left = sensor_data.get("ir_left", 1)
        ir_right = sensor_data.get("ir_right", 1)
        await react_to_obstacle(distance, ir_left, ir_right)
        await asyncio.sleep(1)

async def dance_routine():
    dance_moves = ["forward", "left", "forward", "right"]
    for move in dance_moves:
        await control_robot(move)
        await asyncio.sleep(1)
    await control_robot("stop")

async def express_emotion(emotion):
    emotions = {
        "happy": "Emotion: Happy!",
        "sad": "Emotion: Sad!",
        "startled": "Emotion: Startled!"
    }
    logging.info(emotions.get(emotion, "Emotion: Neutral"))

async def idle_behavior():
    while True:
        action = random.choice(["forward", "left", "right", "stop"])
        await control_robot(action)
        await asyncio.sleep(random.randint(1, 3))
        await control_robot("stop")

async def main():
    logging.info("Starting enhanced robot behavior script")
    # Start environmental monitoring in the background
    asyncio.create_task(monitor_environment())
    # Example of other behaviors
    await asyncio.gather(
        idle_behavior(),
        dance_routine()
    )

if __name__ == "__main__":
    asyncio.run(main())
