Introducing BB1: The Intelligent Companion Robot
BB1 is more than just a robot; it's your intelligent companion designed to make life easier, safer, and a whole lot more fun. Whether you're a tech enthusiast or just someone who appreciates a smart and helpful assistant, BB1 has something to offer. Here's why BB1 is the perfect addition to your home or workspace.

A Brain That Learns and Adapts
At the heart of BB1 is a Raspberry Pi 4, the brain that drives everything. With advanced AI capabilities, object recognition, and voice interaction, BB1 can learn from its environment and adapt to your needs. It uses TensorFlow to detect faces, objects, and even follow you around the house, making it a true companion.

Effortless Mobility and Navigation
BB1's Mobile Unit, powered by an ESP32, gives it the ability to move smoothly and intelligently. With tank treads for traction, ultrasonic sensors for obstacle detection, and an integrated 6-axis gyro, BB1 can navigate through your home without bumping into things. It's perfect for bringing you a drink, following you to the bedroom, or just exploring on its own.

Your Personal Assistant with a Personality
BB1 isn't just functional; it's full of personality. With a 5-inch screen "face," it can show emotions, respond to voice commands, and even express itself through lights and sounds. Need BB1 to find someone? Just ask. Want it to guard the chicken coop? It's on it. This robot doesn't just do what you ask—it does it with style.

Seamless Communication and Control
The Pi 4 acts as an MQTT broker, allowing BB1 to communicate with other chips and sensors seamlessly. Whether it's controlling the Mobile Unit, activating the Streaming Unit, or coordinating the Vocal Unit, BB1 can manage it all. This seamless communication ensures that BB1 stays responsive and adaptable.

Advanced Features in a Compact Package
Despite its compact size, BB1 packs in some serious tech. It has a streaming unit to let you see what it sees, a vocal unit for voice recognition and text-to-speech, and a buzzer unit for simple alerts and lights. And there's more—an additional sensory unit for temperature, humidity, GPS, and compass data is in the works.

Perfect for Everyone
Whether you're a robot enthusiast or just someone looking for a smart helper, BB1 delivers. It's versatile enough for tech lovers to experiment with and user-friendly enough for anyone to enjoy. With BB1, you're not just getting a robot—you're getting a companion that learns, adapts, and makes life more interesting.


AI WROTE THIS------------------

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
