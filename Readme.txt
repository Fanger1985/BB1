4/29/2024 UPDATE

Current State of the Robot:

Brainy Core: A neural network on a Raspberry Pi handles real-time image and sensor data processing for navigation and environment interaction.
ESP32 Scout: This nimble module collects sensor data on the fly, making it super responsive to its immediate surroundings.
Visionary Eyes: It’s got computer vision through OpenCV, allowing it to perceive and react to the visual elements around it.
Servo Savvy: Precise movement control with pigpio, making sure it moves with the grace of a dancer... or at least tries to.
Async Agility: Asyncio and aiohttp for swift, non-blocking communication with the ESP32, keeping everything running smoother than a fresh jar of Skippy.
Environmental Mapping: It’s smart enough to map its environment and remember where it’s been, like a mini explorer charting new lands.
Upcoming Features:

Voice Commander: Pi Zero W integration for voice-to-text capabilities, turning spoken commands into actions.
Advanced Sensor Suite: Plans to incorporate next-level sensors for gesture, distance, luminosity, and orientation to give our bot sensory superpowers.
ChatGPT Co-Pilot: Leveraging language model APIs for complex command parsing and decision-making, making the bot even more interactive and intelligent.
Learning and Adapting: Aiming to give it some learning chops so it can adapt over time and get better at understanding and executing tasks.

----

BB1 Autonomous Robot Project
Welcome to the GitHub repository for the BB1 Autonomous Robot, a sophisticated project that combines the power of both the ESP32 and Raspberry Pi 4 to create a highly interactive and responsive robot capable of navigating and interacting with its environment intelligently.

Project Overview
The BB1 Robot project utilizes a dual-controller architecture where the ESP32 handles low-level sensor interactions and motor control, while the Raspberry Pi 4 serves as the "brain" for processing complex algorithms, making decisions, and handling advanced tasks like image processing.

Components
ESP32: Acts as the "lower brain" managing direct sensor readings, motor actions, and basic navigational tasks.
Raspberry Pi 4: Functions as the "upper brain" handling complex processing, decision-making, environmental mapping, and potentially advanced vision tasks using attached cameras.
Sensors: A mix of ultrasonic, infrared, and possibly other sensors like LIDAR or IMUs, providing data to the ESP32.
Camera: A USB camera connected to the Raspberry Pi for visual data processing and additional navigational aids.
Motors: Controlled by the ESP32 for movement based on commands from either its own logic or the Raspberry Pi.
How It All Works Together
Sensor Data Handling: The ESP32 collects data from various sensors and executes basic navigation tasks. It can operate independently for straightforward tasks or when rapid responses are required.
Data Processing and Decision Making: The Raspberry Pi processes complex data, runs algorithms like A*, and makes decisions about the robot's actions, particularly in complex environments or tasks requiring advanced reasoning.
Communication: The two systems communicate via HTTP requests, with the ESP32 acting as a server and the Raspberry Pi as a client. This setup allows for flexible control schemes and easy integration of additional functionalities.
Why It's Cool
BB1 represents a unique blend of technologies that allows it to perform tasks beyond the capabilities of typical hobby robots:

Dual-Brain Architecture: This setup allows BB1 to handle a range of tasks from simple to complex without overloading the processing capabilities of either the ESP32 or Raspberry Pi.
Adaptive Behavior: BB1 can switch control schemes dynamically based on the task complexity and processing needs, ensuring optimal responsiveness and smart energy use.
Advanced Navigation: With integrated pathfinding algorithms and potential camera-based vision systems, BB1 can navigate complex environments more effectively than many other robots.
Abilities and Behaviors
With Upper Brain (Raspberry Pi)
Advanced Pathfinding: Uses A* algorithm for efficient navigation around obstacles.
Environmental Mapping: Can map its environment in detail and remember it for future navigation, even after being powered down.
Image Processing: Uses camera input for additional navigational help and potential interaction with the environment (e.g., object recognition).
Without Upper Brain (ESP32 Alone)
Basic Navigation: Handles simple movement tasks using sensor data.
Obstacle Avoidance: Can autonomously avoid obstacles using ultrasonic and infrared sensors.
Direct Control: Can be controlled directly via a simple web interface or even a dedicated app.
Controls and Interaction
Control over BB1 can be switched dynamically between the Raspberry Pi and the ESP32 based on the task complexity and processing requirements. This makes BB1 highly versatile and capable of adapting to various scenarios and tasks.


System Communication Architecture
HTTP-Based Command Interface
The BB1 Robot employs a sophisticated communication system that leverages HTTP protocols to enable effective command and control interactions between the Raspberry Pi ("upper brain") and the ESP32 ("lower brain"). This system is designed to optimize the division of labor between handling simple, immediate tasks and processing complex operations requiring more computational power.

ESP32 as HTTP Server
The ESP32 operates as an HTTP server, which means it is set up to listen for HTTP requests sent from the Raspberry Pi. This setup allows the Raspberry Pi to issue commands to the ESP32 dynamically, based on the decision-making processes it carries out:

Endpoint Configuration: The ESP32 exposes various endpoints that correspond to specific actions and sensor data requests, such as moving forward, turning, stopping, or fetching the current sensor readings.
Request Handling: When the ESP32 receives a request, it parses the command, executes the required action (such as initiating motor movements or reading a sensor), and then responds with the outcome. This response might include the status of the action and any requested data.
Response Data: Data sent back from the ESP32 can include success or failure messages, sensor data, or confirmation of movement. This information is critical for the Raspberry Pi to adjust its strategies or update the environmental map.
Raspberry Pi as HTTP Client
On the other side, the Raspberry Pi acts as the HTTP client. It sends requests to the ESP32 and processes the responses:

Command Dispatch: Utilizing its more substantial processing capabilities, the Raspberry Pi computes the required actions, like navigation paths or response to environmental changes, and sends corresponding HTTP requests to the ESP32.
Data Handling: The Raspberry Pi handles heavier data processing tasks, such as analyzing sensor data received from the ESP32 or integrating images from a camera. It uses this information to make informed decisions about subsequent actions, which are then sent as commands back to the ESP32.
Advantages of Using HTTP
Flexibility: HTTP is a widely supported protocol that allows the system to be easily expanded with additional functionalities or integrated with other systems and interfaces.
Reliability: HTTP operates over TCP/IP, ensuring that commands are reliably delivered and responses are verified, contributing to the robustness of the robot’s operational capabilities.
Ease of Debugging and Testing: Using HTTP makes it easier to test and debug the system since requests can be sent from standard web browsers or tools like Postman.
Technical Stack and Implementation
Networking Setup: The ESP32 is configured with a Wi-Fi connection, making it accessible on the local network. It uses a simple web server, often developed with libraries like ESPAsyncWebServer, to handle HTTP requests.
Security Considerations: While the primary setup focuses on functionality, considerations around securing the HTTP endpoints can be implemented to ensure that the system is not open to unauthorized access, especially when deployed in environments with broader network access.
This communication architecture not only maximizes the computational strengths of both the Raspberry Pi and ESP32 but also ensures that BB1 can perform a wide range of tasks efficiently, from simple direct controls to complex autonomous behaviors based on real-time data analysis and decision-making.



---------------------------------------------------------------------------------------------------------------------------------------------------

BB1 stands out because it shows that you don't need expensive high-end chips to create a capable and interactive robot. While big names like Tesla and Boston Dynamics might dominate the high-tech scene, BB1's design and construction demonstrate that you can do amazing things with low-power, cost-effective components. Here's why that's important:

Cost-Effective Innovation: By using budget-friendly parts like the ESP32 and Raspberry Pi 4, BB1 proves that creativity and ingenuity can go a long way. You don't need to break the bank to build a robot that can follow you, recognize faces, and even stream video. This approach makes robotics more accessible to hobbyists, educators, and DIY enthusiasts.
Energy Efficiency: Low-power chips like the ESP32 are not only affordable but also energy-efficient. This means BB1 can operate for extended periods without draining batteries or requiring complex power management systems. It's a sustainable approach that aligns with modern trends in energy conservation and eco-friendliness.
Resourceful Engineering: The fact that BB1 can do so much with basic components shows resourcefulness and ingenuity. It demonstrates that you can build a robot that learns, adapts, and interacts without needing the latest and greatest hardware. This resourcefulness is an inspiration to those who want to create but might feel limited by budget constraints.
DIY Spirit: BB1 embodies the DIY spirit, encouraging others to think creatively and find innovative solutions with readily available components. It's a reminder that you don't need to be a large corporation to build something cool and functional. This spirit resonates with makers, tinkerers, and anyone who loves creating with their hands.
Customization and Flexibility: Since BB1 uses low-cost components, it's easier to customize and modify. You can experiment with different sensors, attachments, or features without worrying about expensive replacements or high repair costs. This flexibility allows BB1 to evolve and grow with your needs and creativity.






-------------------------------

THESE ARE IN REVERSE ORDER WITH THE NEWEST BEING ON TOP!

4-19-2024
BB1 UPDATE!
Successfully have the Tank Tread ESP32 Communicating with a Buzzer ESP32 which will be attached to more sensors. Both will ultmately be controlled by the pi 4 via mqtt.  
Now have a 3rd W-Rover Board acting as a video streaming server.  kind of laggy tho. something more powerful would be better.

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
