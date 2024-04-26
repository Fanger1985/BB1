import requests
import time

# Base URL for the ESP32 Web server
esp32_base_url = "http://192.168.1.2/"  # Replace with your ESP32's IP address

# Function to send HTTP GET requests to the ESP32
def send_http_get(endpoint):
    try:
        response = requests.get(esp32_base_url + endpoint)
        if response.status_code == 200:
            print(f"Successfully sent GET request to {endpoint}")
            return response.json()  # If response is in JSON format
        else:
            print(f"Error sending GET request to {endpoint}: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Exception during GET request to {endpoint}: {e}")

# Function to send HTTP POST requests to the ESP32
def send_http_post(endpoint, data):
    try:
        response = requests.post(esp32_base_url + endpoint, json=data)
        if response.status_code == 200:
            print(f"Successfully sent POST request to {endpoint}")
        else:
            print(f"Error sending POST request to {endpoint}: {response.status_code}")
    except requests.exceptions.RequestException as e:
        print(f"Exception during POST request to {endpoint}: {e}")

# Function to control the robot
def control_robot(command):
    # Define HTTP endpoints for control commands
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
        send_http_get(endpoints[command])
    else:
        print(f"Invalid command: {command}")

# Function to retrieve sensor data
def get_sensor_data():
    sensor_data = send_http_get("/sensors")
    if sensor_data:
        print("Sensor Data:", sensor_data)

# Function to handle AI-related operations (placeholder)
def ai_operations():
    print("Placeholder for AI-based operations")
    # Insert TensorFlow/AI code here in the future

# Perform a figure-8 pattern
def figure_8():
    # Forward for 1 second
    control_robot("forward")
    time.sleep(1)
    control_robot("stop")
    
    # Spin left for 1 second
    control_robot("left")
    time.sleep(1)
    control_robot("stop")
    
    # Forward for 1 second
    control_robot("forward")
    time.sleep(1)
    control_robot("stop")
    
    # Spin right for 1 second
    control_robot("right")
    time.sleep(1)
    control_robot("stop")

# Main script logic
if __name__ == "__main__":
    print("Starting Pi4 control script for the ESP32-based robot")

    # Retrieve initial sensor data
    get_sensor_data()
    
    # Perform the figure-8 pattern
    figure_8()

    # Placeholder for AI-related operations
    ai_operations()

    print("Pi4 control script execution complete.")
