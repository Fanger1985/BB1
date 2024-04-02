THESE ARE IN REVERSE ORDER WITH THE NEWEST BEING ON TOP!


4-02-2024
BB1 Update!
BB1 Looks like A mini Wall-E / Johnny 5 Hybrid baby.
BB1 Is moving around! Face just looks around right now and Wheels are basically just running blind but the nomving around is a big accomplishment for me!



Rasberry Pi 4 (Main Brain) (MQTT Broker)
     - 5" Screen
          The Screen will be used to display a HTML/CSS/JS page that has a simple graphic of an eye.  Utilizes tensorflow light to track faces and look at people.  This is part of "smart mode" since the AI resoudces can be intensive on the Pi4.  I plan to have the dumb ai pick up on cues that would activate the smart ai when needed.  
     - USB Camera
     - USB Microphone
     - Aux Port Speaker (Not sure if i want to go this route yet or rely on the Pi Zero)

  - BrainCraft Hat Attached to Pi 4 (at this time attached but not being utilzied. 
         - Currently Equipped :
         - 2 Servos 
             - Can probably use these in the face for personality eye brows?
             - Can probably utilize these for a Sonar style UltraSonic panning system saving the tank tread from doing the looking around.
             - To look big and mean somehow.
         - 2 Speakers 
         - Fan

Freenove ESP32-WROOM on Big Beautiful Freenove Breakout Board 
      - Front Ultrasonic Senor
      - Rear Ultarsonic Sensor  -or-  2 Rear IR sennsors
      - 2 x DRV8871 DC Motor Driver With H Bridge (So Reversing can happen)
      - 2 Hall sensors for tracking motor speed.  
      - 2 Motors are currenlty 150 RPM 9V DC for powering 2 tank treads.



I modularized the Robot for easier upgrading in the future .  The robot will now host its own Access Point with each component (Arms , Wheels, Sensors, Etc) being handled by an ESP32 Chip.  These components will send and recieve info to the brain via MQTT.   - Head has a ribbon strip going to his backpack which is GPIO ribbon connected to an Adafruit Braincraft Hat.

Rasberry Pi4 Main Brain - This is in the Robots Head .  This board controls the Face Screen visuals as well as image detection & classification.  This board is also the MQTT broker for communicating with the rest of the robot.  

Currently using a Freenove ESP32 WRoom (Which will be a independently acting unless instructed otherwise by the Pi Brain to do special commands) for the main driving unit of the Robot.  Currently the 2 Motor Drivers, 2 Hall sensors,  2 Ultrasonic sensors (Soon to be 1 ultrasonic in front and 2 IR Obstacle Sensors in the rear at 7Oclock and 5O'clock) .  I had to write a custom analog function to control the PWM to make this board work.  It is currently working however motors engage at almost full power instantly and the bot is doing wheelies.  LEDC control of PWM does not compile onto the FreeNove W-Room Boards / i might not know how to do it.  
   - Rewrote this including LEDC PWN control of the motors for Arduino Nano ESP32 and it compiled successfully, currently untested with Arduino.

Pi Zero -W


TO DO:

PLANNED Rasberry Pi Zero PLANNED - Id like to implement this specifically to handle Voice Recognition AI. Ideally conversationally but to start we would be utilizing as a way to pick up certain words
to forward to the MQTT broker to determine bot behavior.  An Upgrade Module for the future.   This would allow for constant smart talk even if we are in dumb ai mode. (no visual response on screen and ran by esp32s on motion.
    - Independnet Microphone 
    - Speech Recognition AI 
    - Independent Speaker


Additional ESP32 Control For : 
   - Fancy Lighting stuff
   - Additional Environmental Sennsors 
        - Climate
        - Orientation
        - Accelerometer 
        - GPS Module
        - Line Tracking (For Deer Patroll)
        - Cliff Detection
        - Remote COntrol.. ?



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
