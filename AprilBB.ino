#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h> // Make sure to include this

// Network credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// Motor pin definitions
#define IN1_LEFT 19
#define IN2_LEFT 21
#define IN1_RIGHT 5
#define IN2_RIGHT 18

// Sensor pin definitions
#define TRIG_PIN 16
#define ECHO_PIN 17
#define IR_LEFT 13 //REAR LEFT FACING AWAY FROM ROBOT
#define IR_RIGHT 4 // REAR RIGHT FACING AWAY FROM ROBOT

// Global variables for sensor readings and motor speed
long duration;
int distance;
int IR_Left_Value;
int IR_Right_Value;
volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
int lastLeftPulseCount = 0;
int lastRightPulseCount = 0;
int speedLeft = 255;  // Max speed
int speedRight = 255; // Max speed
volatile int leftHallPulseCount = 0;
volatile int rightHallPulseCount = 0;

// Web server on port 80
WebServer server(80);

// Forward declarations of ISR functions
void IRAM_ATTR onLeftEncoder();
void IRAM_ATTR onRightEncoder();
void IRAM_ATTR onLeftHallSensor();
void IRAM_ATTR onRightHallSensor();

// Define ISR implementations
void IRAM_ATTR onLeftEncoder() {
    leftPulseCount++;
}

void IRAM_ATTR onRightEncoder() {
    rightPulseCount++;
}

void IRAM_ATTR onLeftHallSensor() {
    leftHallPulseCount++;
}

void IRAM_ATTR onRightHallSensor() {
    rightHallPulseCount++;
}

// Global state variable to determine control mode
volatile bool isManualControl = false;

// Define all function prototypes here
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void stopMotors();
void smoothStartMotors();
int getUltrasonicDistance();
String controlPage();

unsigned long startTime = 0;  // Store the startup time
bool initialDelayComplete = false;  // Track if the delay is completed

void setup() {
    // Initialize serial communication and record the start time
    Serial.begin(9600);
    startTime = millis();

    // Initialize motor and sensor pins
    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);

    // Setup Hall sensor pins
    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(32, INPUT_PULLUP);
    pinMode(33, INPUT_PULLUP);

    // Attach interrupts for Hall sensors
    attachInterrupt(digitalPinToInterrupt(34), onLeftHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(35), onLeftHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(32), onRightHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(33), onRightHallSensor, RISING);

    // Establish WiFi connection
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi network.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Introduce an initial delay before starting operations
    Serial.println("Preparing to start operations...");
    delay(5000); // Gives time for initialization

    // Setup Web server routes
    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", controlPage());
    });

server.on("/forward", HTTP_GET, []() {
    isManualControl = true; // Switch to manual control
    moveForward(); // Move forward
    server.send(200, "text/plain", "Moving forward");
});

server.on("/backward", HTTP_GET, []() {
    isManualControl = true; // Switch to manual control
    moveBackward(); // Move backward
    server.send(200, "text/plain", "Moving backward");
});

server.on("/left", HTTP_GET, []() {
    isManualControl = true; // Switch to manual control
    spinLeft(); // Spin left
    server.send(200, "text/plain", "Turning left");
});

server.on("/right", HTTP_GET, []() {
    isManualControl = true; // Switch to manual control
    spinRight(); // Spin right
    server.send(200, "text/plain", "Turning right");
});

server.on("/stop", HTTP_GET, []() {
    isManualControl = true; // Switch to manual control
    stopMotors(); // Stop all motors
    server.send(200, "text/plain", "Stopping");
});

server.on("/auto", HTTP_GET, []() {
    isManualControl = false; // Switch to autonomous mode
    server.send(200, "text/plain", "Switched to auto mode");
});


    server.on("/explore", HTTP_GET, []() {
        exploreEnvironment();
        server.send(200, "text/plain", "Exploration mode activated");
    });

    server.on("/dance", HTTP_GET, []() {
        danceRoutine();
        server.send(200, "text/plain", "Dance sequence activated");
    });

    server.on("/expressEmotion", HTTP_GET, []() {
        expressEmotion("happy");
        server.send(200, "text/plain", "Emotion expressed");
    });

    server.on("/sensors", HTTP_GET, []() {
        IR_Left_Value = digitalRead(IR_LEFT);
        IR_Right_Value = digitalRead(IR_RIGHT);
        distance = getUltrasonicDistance();

        String sensorData = "{";
        sensorData += "\"ir_left\":" + String(IR_Left_Value) + ",";
        sensorData += "\"ir_right\":" + String(IR_Right_Value) + ",";
        sensorData += "\"distance\":" + String(distance);
        sensorData += "}";

        server.send(200, "application/json", sensorData);
    });

    server.on("/expressEmotion", HTTP_POST, handleExpressEmotion); // Separate function for POST handling

    server.begin(); // Start the HTTP server
    Serial.println("HTTP server started. Ready for commands.");
}


void smoothStartMotors() {
    int targetSpeed = 255;
    int increment = 10;

    for (int speed = 0; speed <= targetSpeed; speed += increment) {
        // Use direct GPIO manipulation for PWM-like behavior
        for (int i = 0; i < 255; i++) {
            digitalWrite(IN1_LEFT, i < speed ? HIGH : LOW);
            digitalWrite(IN1_RIGHT, i < speed ? HIGH : LOW);
            delayMicroseconds(1000); // This creates the PWM effect
        }
        delay(20); // Delay between increments to smooth out the transition
    }
}


void loop() {
    server.handleClient();  // Handle client requests to the web server

    static unsigned long lastReportTime = 0;
    const unsigned long reportInterval = 1000;  // Report every 1000 milliseconds (1 second)

    if (!isManualControl) {
        // Autonomous behavior
        autoMove();
    }

    // Check if it's time to report the IR sensor status and distance
    if (millis() - lastReportTime >= reportInterval) {
        lastReportTime = millis();  // Update the last report time

        // Read IR sensor values
        IR_Left_Value = digitalRead(IR_LEFT);
        IR_Right_Value = digitalRead(IR_RIGHT);

        // Get the current distance from the ultrasonic sensor
        distance = getUltrasonicDistance();

        // Report IR sensor status and ultrasonic distance
        Serial.print("IR Left Value: ");
        Serial.print(IR_Left_Value);
        Serial.print(" | IR Right Value: ");
        Serial.print(IR_Right_Value);
        Serial.print(" | Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}

void handleExpressEmotion() {
    String body = server.arg("plain");
    DynamicJsonDocument doc(256); // Declare a document for deserialization
    deserializeJson(doc, body); // Deserialize incoming JSON data

    String emotion = doc["emotion"]; // Extract emotion from the JSON

    // Determine the expression based on emotion
    if (emotion == "happy") {
        expressEmotion("happy");
    } else if (emotion == "sad") {
        expressEmotion("sad");
    } else if (emotion == "startled") {
        expressEmotion("startled");
    }

    server.send(200, "text/plain", "Emotion received"); // Respond to the client
}
void autoMove() {
    if (!initialDelayComplete) {
        // Check if 15 seconds have passed since startup
        if (millis() - startTime >= 15000) {
            initialDelayComplete = true;  // Set flag to avoid further checks
        } else {
            // If still in the initial delay, do nothing to avoid auto movement
            return;
        }
    }

    // After the initial delay, continue with the regular auto-move logic
    IR_Left_Value = digitalRead(IR_LEFT);
    IR_Right_Value = digitalRead(IR_RIGHT);
    distance = getUltrasonicDistance();

    if (distance > 100) {
        idleWander();  // If there's plenty of space, wander around
    } else if (distance < 30) {
        reactToCloseObstacle();  // If too close to an obstacle, back off
    } else {
        cautiousApproach();  // If moderate distance, approach cautiously
    }
}
unsigned long idleStartTime = 0;  // Track idle start time
bool isIdle = false;  // Flag to track if BB1 is idling

void idleWander() {
    int action = random(0, 100);  // Random action selector

    if (!isIdle) {
        if (action < 20) {  // 20% chance to enter idle state
            idleStartTime = millis();
            isIdle = true;  // BB1 is now idling
            return;  // Do nothing to simulate sitting still
        } else if (action < 50) {  // 30% chance to move forward
            moveForward();
            delay(500);  // A small delay for movement, avoid long blocking
            stopMotors();
        } else if (action < 75) {  // 25% chance to spin left
            spinLeft();
            delay(300);
            stopMotors();
        } else {  // 25% chance to spin right
            spinRight();
            delay(300);
            stopMotors();
        }
        expressEmotion("happy");
    } else {
        // If idling, check if enough time has passed
        if (millis() - idleStartTime >= random(1000, 5000)) {
            isIdle = false;  // Reset idle state
        }
    }
}


void reactToCloseObstacle() {
    // Simple backup and quick turn
    moveBackward();
    delay(random(500, 1000)); // Quick backup
    stopMotors();
    
    // Random quick turn after backup
    if (random(0, 2)) {
        spinLeft();
        delay(200); // Quick spin
    } else {
        spinRight();
        delay(200); // Quick spin
    }
    stopMotors();
}
void expressEmotion(String emotion) {
    if (emotion == "happy") {
        // Play a happy sound or flash a bright LED
        Serial.println("BB1 is happy!");
        // Play sound or blink LEDs
    } else if (emotion == "startled") {
        Serial.println("BB1 is startled!");
        // Play a startled sound or blink LEDs in a pattern
    }
}
// Example function on the mobile unit to send HTTP requests to the buzzer unit
void triggerEmotion(String emotion) {
    HTTPClient http;
    const char* buzzerUnitIP = "192.168.1.3";  // IP of the buzzer unit
    String serverUrl = "http://" + String(buzzerUnitIP) + "/expressEmotion";  // Endpoint on the buzzer unit
    
    http.begin(serverUrl);  // Start the HTTP session
    http.addHeader("Content-Type", "application/json");

    // Create a JSON object with the emotion to be triggered
    DynamicJsonDocument doc(256);
    doc["emotion"] = emotion;

    String jsonData;
    serializeJson(doc, jsonData);

    int httpCode = http.POST(jsonData);  // Send the POST request with the emotion data

    if (httpCode == 200) {
        Serial.println("Emotion triggered successfully!");
    } else {
        Serial.print("Failed to trigger emotion, HTTP code: ");
        Serial.println(httpCode);
    }

    http.end();  // End the HTTP session
}

void cautiousApproach() {
    int distance = getUltrasonicDistance();
    if (distance > 100) {
        moveForward(); // Safe to move forward
        expressEmotion("happy");
    } else if (distance < 30) {
        reactToCloseObstacle(); // React to close obstacle
        expressEmotion("startled");
    } else {
        idleWander(); // Cautious wandering
    }
}

void exploreEnvironment() {
    IR_Left_Value = digitalRead(IR_LEFT);
    IR_Right_Value = digitalRead(IR_RIGHT);
    distance = getUltrasonicDistance();

    // Analyze the environment and make decisions
    if (distance > 100) {  // If there's a lot of space ahead, go explore it
        moveForward();
        delay(1000);  // Move forward for a bit
    } else if (distance < 30) {  // Too close to something, need to decide direction
        handleFrontObstacle();  // Handle it appropriately
    }

    // Check rear sensors for obstacles when choosing to back up
    if (IR_Left_Value == 0 || IR_Right_Value == 0) {
        handleRearObstacle();
    } else if (random(0, 2) == 0) {  // Randomly decide to back up if all clear
        moveBackward();
        delay(500);
    }

    // Occasionally spin around to scan the area
    if (random(0, 10) < 2) {
        spinLeft();
        delay(1000);
    } else if (random(0, 10) < 2) {
        spinRight();
        delay(1000);
    }
}


String controlPage() {
    String html = R"(
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
  body { 
    font-family: Arial, sans-serif; 
    background: #e0e5ec; 
    display: flex; 
    flex-direction: column; 
    align-items: center; 
    height: 100vh; 
    margin: 0; 
  }
  .button { 
    border: none; 
    border-radius: 12px; 
    padding: 20px 40px; 
    font-size: 16px; 
    color: #fff; 
    cursor: pointer; 
    outline: none; 
    margin: 10px; 
  }
  .stop { 
    background: #ff4136; 
  }
  .control { 
    background: #7fdbff; 
  }
  .special { 
    background: #85144b; 
  }
  .button:active { 
    color: #000; 
  }
  #controlGrid { 
    display: grid; 
    grid-template-rows: repeat(3, auto); 
    grid-template-columns: repeat(3, auto); 
    justify-content: center; 
    align-items: center; 
    gap: 10px; 
  }
  #autoButton { 
    grid-column: 1; 
    grid-row: 1; 
  }
  #exploreButton { 
    grid-column: 3; 
    grid-row: 1; 
  }
  #forwardButton { 
    grid-column: 2; 
    grid-row: 1; 
  }
  #leftButton { 
    grid-column: 1; 
    grid-row: 2; 
  }
  #stopButton { 
    grid-column: 2; 
    grid-row: 2; 
  }
  #rightButton { 
    grid-column: 3; 
    grid-row: 2; 
  }
  #backwardButton { 
    grid-column: 2; 
    grid-row: 3; 
  }
  #danceButton { 
    grid-column: 1; 
    grid-row: 3; 
  }
  #expressEmotionButton { 
    grid-column: 3; 
    grid-row: 3; 
  }
</style>
</head>
<body>
<h1>Robot Control Interface</h1>
<div id="controlGrid">
  <button class="button special" id="autoButton" onclick='sendCommand("/auto"); return false;'>Auto</button>
  <button class="button special" id="exploreButton" onclick='sendCommand("/explore"); return false;'>Explore</button>
  <button class="button control" id="forwardButton" onmousedown='sendCommand("/forward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Forward</button>
  <button class="button control" id="leftButton" onmousedown='sendCommand("/left"); return false;' onmouseup='sendCommand("/stop"); return false;'>Left</button>
  <button class="button stop" id="stopButton" onclick='sendCommand("/stop"); return false;'>Stop</button>
  <button class="button control" id="rightButton" onmousedown='sendCommand("/right"); return false;' onmouseup='sendCommand("/stop"); return false;'>Right</button>
  <button class="button control" id="backwardButton" onmousedown='sendCommand("/backward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Backward</button>
  <button class="button special" id="danceButton" onclick='sendCommand("/dance"); return false;'>Dance</button>
  <button class="button special" id="expressEmotionButton" onclick='sendCommand("/expressEmotion"); return false;'>Express Emotion</button>
</div>
<iframe id="logFrame" srcdoc="<p>Command log initialized...</p>"></iframe>
<script>
function sendCommand(command) {
    var xhr = new XMLHttpRequest(); // Create a new XMLHttpRequest object
    xhr.open('GET', command, true); // Open the GET request to the command endpoint
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE) { // Check if the request is complete
            if (xhr.status == 200) { // Check if the status is OK
                var logFrame = document.getElementById("logFrame");
                var logDocument = logFrame.contentDocument || logFrame.contentWindow.document;
                logDocument.body.innerHTML += "<p>" + command + " command executed.</p>";
            } else {
                console.error("Failed to execute command: " + xhr.status); // Log an error if the request fails
            }
        }
    };
    xhr.send(); // Send the GET request
    return false; // Prevent default action, stop navigation
}

</script>
</body>
</html>
)";
    return html;
}


// Define motor control and sensor reading functions...
void moveForward() {
    Serial.println("Moving forward...");
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    digitalWrite(IN1_RIGHT, HIGH);
    digitalWrite(IN2_RIGHT, LOW);
}

void moveBackward() {
    Serial.println("Checking for obstacles before moving backward...");
    IR_Left_Value = digitalRead(IR_LEFT);
    IR_Right_Value = digitalRead(IR_RIGHT);

    if (IR_Left_Value == 0 || IR_Right_Value == 0) {
        Serial.println("Obstacle detected at rear, stopping...");
        stopMotors();  // Stop to prevent collision
    } else {
        Serial.println("Rear path clear, moving backward...");
        digitalWrite(IN1_LEFT, LOW);
        digitalWrite(IN2_LEFT, HIGH);
        digitalWrite(IN1_RIGHT, LOW);
        digitalWrite(IN2_RIGHT, HIGH);
    }
}
void spinLeft() {
    Serial.println("Spinning left...");
    // Spin left: Right motor forward, Left motor backward
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, HIGH);
    digitalWrite(IN1_RIGHT, HIGH);
    digitalWrite(IN2_RIGHT, LOW);
}

void spinRight() {
    Serial.println("Spinning right...");
    // Spin right: Left motor forward, Right motor backward
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, HIGH);
}

void stopMotors() {
    Serial.println("Stopping motors...");
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, LOW);
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, LOW);
}

int getUltrasonicDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = duration * 0.034 / 2;
    Serial.print("Distance measured: ");
    Serial.println(distance);
    return distance;
}

void handleFrontObstacle() {
    Serial.println("Front obstacle detected, stopping...");
    stopMotors();
    delay(1500);
    // Additional check for head clearance
    if (distance < 40) {  // Assuming 50 cm is a safe stopping distance before head impact
        Serial.println("Obstacle might hit the head, extra precautions taken.");
        // Implement additional safety measures here
    } else {
        Serial.println("Lower obstacle detected, normal avoidance maneuver.");
    }
    Serial.println("Checking rear path before reversing...");
    if (IR_Left_Value == 0 || IR_Right_Value == 0) {
        Serial.println("Obstacle detected at rear, spinning instead of reversing...");
        if (IR_Left_Value == 0) {
            spinRight();
        } else {
            spinLeft();
        }
    } else {
        Serial.println("Rear path clear, reversing...");
        moveBackward();
        delay(2000);
        stopMotors();
    }
}

void handleRearObstacle() {
    Serial.println("Rear obstacle detected, adjusting course...");
    if (IR_Left_Value == 0) {
        spinRight();
    } else if (IR_Right_Value == 0) {
        spinLeft();
    }
    delay(1500);
    moveForward();
}

bool checkSpaceForDance() {
    int frontDistance = getUltrasonicDistance();
    Serial.print("Front distance: ");
    Serial.println(frontDistance);
    if (frontDistance < 100) { // Needs at least 100 cm of free space in front
        Serial.println("Not enough space to dance :(");
        return false;
    }
    return true;
}

void danceRoutine() {
    Serial.println("Starting dance routine...");

    // Dance move 1: Figure 8 pattern
    Serial.println("Dance move 1: Figure 8 pattern");
    moveForward();
    delay(1000);  // Move forward for a bit
    spinLeft();
    delay(1000);  // Spin left for 1 second
    moveForward();
    delay(1000);  // Continue forward
    spinRight();
    delay(1000);  // Spin right to complete the figure 8
    stopMotors();

    // Dance move 2: Spinning with LED placeholder
    Serial.println("Dance move 2: Extended spin with LED effect.");
    spinLeft();
    Serial.println("blink blink");  // Placeholder for LED effect
    delay(3000);  // Spin for 3 seconds
    stopMotors();

    // Dance move 3: Directional jiggle
    Serial.println("Dance move 3: Jiggle back and forth");
    for (int i = 0; i < 5; i++) {
        moveForward();
        Serial.println("blink blink");  // Placeholder for LED effect
        delay(200);
        moveBackward();
        delay(200);
    }

    // Dance move 4: Circle spin
    Serial.println("Dance move 4: Circle spin");
    spinRight();
    delay(3000);  // Longer spin to the right
    stopMotors();

    Serial.println("Dance routine complete!");
    server.send(200, "text/plain", "Dance complete");
    isManualControl = false;  // Reset manual control state after dancing
}

#include <vector>  // To use vectors for dynamic arrays

std::vector<int> distances;  // Record distances as he moves
std::vector<String> actions;  // Record actions taken at each point

void recordPath() {
    distances.push_back(getUltrasonicDistance());  // Record distance
    actions.push_back("moveForward");  // Record action taken
}


#include <map>  // For storing a simple map of the environment

std::map<int, int> environmentMap;  // Mapping distance and action
int currentPosition = 0;  // Track BB1's position

void updateMap() {
    int currentDistance = getUltrasonicDistance();
    environmentMap[currentPosition] = currentDistance;  // Store the distance at the current position
    currentPosition++;  // Move to the next position
}


void navigate() {
    int currentDistance = getUltrasonicDistance();
    if (currentDistance < 30) {  // Close to an obstacle
        if (environmentMap[currentPosition - 1] < 30) {
            // Take a different path based on learned environment
            spinRight();  // Example of adapting behavior
            delay(500);
        } else {
            reactToCloseObstacle();  // Default behavior if no learned path
        }
    } else {
        moveForward();  // Continue moving forward
    }
}

