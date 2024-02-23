2-24-2024
Autonomous Bot Project

Overview
This project involves creating an autonomous bot powered by a Raspberry Pi, equipped with motor control, ultrasonic sensors for obstacle detection, and a USB camera for object detection using AI. The bot can navigate its environment, avoid obstacles, and recognize specific objects, making it smarter and more interactive.

Features
Motor Control: Drives two motors using GPIO pins to move forward, reverse, and turn.
Ultrasonic Sensors: Utilizes front and back ultrasonic sensors to detect obstacles and avoid collisions.
AI-Powered Vision: Integrates a USB camera with OpenCV and a pre-trained MobileNet SSD model for real-time object detection.

New Additions
Object Detection: The bot can now use its USB camera to identify objects in its path. It uses a pre-trained MobileNet SSD model within OpenCV to recognize common objects and react accordingly (e.g., stopping when it detects a person).
Enhanced Autonomy: With the addition of object detection, the bot's autonomy level is significantly improved, allowing for smarter navigation and interaction with its environment.
Setup and Installation
Hardware Setup: Ensure your Raspberry Pi is connected to the motor controller, ultrasonic sensors, and USB camera as per the circuit diagram (not provided here).
Software Requirements: Install the necessary libraries with the following commands:
bash

pip install RPi.GPIO opencv-python-headless
Model Download: Download the MobileNet SSD model and its corresponding prototxt file from the OpenCV repository and place them in your project directory.
Running the Script
Execute the main script to start the bot:

How It Works
The bot continually checks for obstacles using ultrasonic sensors and navigates around them.
Simultaneously, it processes the video feed from the USB camera to detect objects.
If an object is recognized (e.g., a person), the bot can perform predefined actions like stopping or turning away, enhancing its interaction with the environment.
Future Enhancements
Implement more complex AI models for better object recognition and decision-making.
Integrate additional sensors for more sophisticated environment sensing and navigation.
Explore the integration of voice commands for an interactive experience.





2-23-2024

Hey Dude ! You probably Gotta Do this!
pip install RPi.GPIO picamera opencv-python-headless

godINC.py is incomplete work in progress : this will be the basic functions when nothing interesting is seen by the robot. It will wander and do its 
own thing until it finds something interesting.

dservo.py  Hey the servo on the braincraft works! "vbq" keyboard keys for function . just for tests.
dservo2.py Hey the servo on the braincraft works! "vbq" keyboard keys for function . just for tests.
