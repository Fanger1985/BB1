#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h> // Make sure to include this
#include <vector>
#include <map>
#include <Wire.h> // I2C communication
#include <MPU6050.h> // MPU6050 library

// Global declarations
std::vector<int> distances;
std::vector<String> actions;
int currentPosition = 0;  // Global position tracker
// Declare 'score' variable
int score = 0;

std::map<int, std::vector<int>> environmentMap;  // Use std::vector<int> for multiple values

// Network credentials
const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

// Motor pin definitions
#define IN1_LEFT 19
#define IN2_LEFT 15//CHANGED FROM 21 MOVE WIRE
#define IN1_RIGHT 5
#define IN2_RIGHT 18

// Sensor pin definitions
#define TRIG_PIN 16
#define ECHO_PIN 17
#define IR_LEFT 13 //REAR LEFT FACING AWAY FROM ROBOT
#define IR_RIGHT 4 // REAR RIGHT FACING AWAY FROM ROBOT

#define PIR_SENSOR_PIN 12 // Change to 14 if needed

// MPU6050 declarations
MPU6050 mpu; // Create MPU6050 object
int16_t ax, ay, az; // Acceleration
int16_t gx, gy, gz; // Gyroscope
// Manual Control State
volatile bool isManualControl = true;

bool isStuck = false; // Flag to track if BB1 is stuck

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

// Forward declarations for additional functions
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void stopMotors();
void smoothStartMotors();
int getUltrasonicDistance();
String controlPage();
void danceRoutine();
void recordPath(int distance, String action);
void updateMap();
void exploreEnvironment();
void autoMove();
void cautiousApproach();
void idleWander();
void reactToCloseObstacle();
void expressEmotion(String emotion);
void adjustBehaviorBasedOnScore();
void calculateScore(bool avoidedObstacle);
void navigate();
void manageMemory();


unsigned long startTime = 0;  // Store the startup time
bool initialDelayComplete = false;  // Track if the delay is completed

void setup() {
    // Initialize serial communication and record the start time
    Serial.begin(9600);
    
    // Initialize I2C for MPU6050
    Wire.begin();  // Initialize I2C communication
    mpu.initialize();  // Initialize the MPU6050
    
    pinMode(PIR_SENSOR_PIN, INPUT); // Set the PIR sensor pin as input
    
    // Set accelerometer range (example: ±8g)
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);

    // Set gyroscope range (example: ±1000°/s)
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

    // Check if MPU6050 is working correctly
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
    } else {
        Serial.println("MPU6050 connection successful.");
    }
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

server.on("/human_detected", HTTP_GET, []() {
    int pirState = digitalRead(PIR_SENSOR_PIN);
    String response = pirState == HIGH ? "Human detected" : "No human detected";
    server.send(200, "text/plain", response);
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
server.on("/getData", HTTP_GET, []() {
    String data = "{ \"distances\": [";
    for (int d : distances) {
        data += String(d) + ",";
    }
    data.remove(data.length() - 1);  // Remove trailing comma
    data += "], \"actions\": [";
    for (String a : actions) {
        data += "\"" + a + "\",";
    }
    data.remove(data.length() - 1);  // Remove trailing comma
    data += "] }";

    server.send(200, "application/json", data);  // Send data in JSON format
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
    
    // Safety checks with MPU6050
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Get accelerometer and gyroscope readings

    if (abs(ax) > 15000 || abs(ay) > 15000 || abs(az) > 15000) { // Check for sudden movement
        Serial.println("Sudden movement detected! Stopping motors.");
        stopMotors(); // Safety measure to avoid damage
    }

    if (!isManualControl) { // Only respond to sensors if in auto mode
        // Check if a human is detected
        int pirState = digitalRead(PIR_SENSOR_PIN);

        if (pirState == HIGH) { // If human is detected
            Serial.println("Human detected! Expressing happiness...");
            expressEmotion("happy"); // Play sound or flash LED

            int followDistance = getUltrasonicDistance(); // Distance to the person

            if (followDistance > 100) { // If the person is far, move forward
                moveForward();
                delay(500); // Short movement delay
            } else if (followDistance < 30) { // If it's too close, stop
                stopMotors(); // Prevent collisions
            }

            // Obstacle detection
            if (followDistance < 30) {
                Serial.println("Obstacle detected, stopping and finding a new path...");
                reactToCloseObstacle(); // Evasive action
            }

            if (followDistance >= 100) {
                Serial.println("Obstacle cleared, resuming following...");
                moveForward(); // Resume following
            }

            updateMap(); // Keep environment map updated
        } else { // If no human is detected
            Serial.println("Exploring the environment...");
            autoMove(); // Continue exploring and mapping
        }
    }

    // Regular sensor data reporting
    static unsigned long lastReportTime = 0;
    const unsigned long reportInterval = 1000; // Report every 1000 milliseconds
    
    if (millis() - lastReportTime >= reportInterval) { // Check for reporting interval
        lastReportTime = millis(); // Update report time
        
        IR_Left_Value = digitalRead(IR_LEFT); // Read IR sensor values
        IR_Right_Value = digitalRead(IR_RIGHT);
        distance = getUltrasonicDistance(); // Get the ultrasonic distance

        Serial.print("IR Left Value: ");
        Serial.print(IR_Left_Value);
        Serial.print(" | IR Right Value: ");
        Serial.print(IR_Right_Value);
        Serial.print(" | Distance: ");
        Serial.print(distance);
        Serial.println(" cm");

        recordPath(distance, "sensor update"); // Record sensor data
        updateMap(); // Keep environment map updated
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
      adjustBehaviorBasedOnScore();
    // Read MPU6050 data for sudden changes
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Check for sudden movements or orientation changes
    if (abs(ax) > 15000 || abs(ay) > 15000 || abs(az) > 15000) {
        Serial.println("Significant acceleration detected! Stopping motors.");
        stopMotors(); // Safety check to prevent damage
        return;
    }

    if (abs(gx) > 15000 || abs(gy) > 15000 || abs(gz) > 15000) {
        Serial.println("Significant angular rate change detected! Stopping motors.");
        stopMotors(); // Safety check for sudden orientation change
        return;
    }

    // Get ultrasonic distance
    int distance = getUltrasonicDistance();

    if (distance > 100) { // If there's plenty of space
        Serial.println("Open space, initiating idle wander...");
        idleWander(); // Idle wander in open spaces
    } else if (distance < 30) { // Close to an obstacle
        Serial.println("Obstacle detected, initiating cautious approach...");
        reactToCloseObstacle(); // Back off and find a new path
        calculateScore(false); // Deduct points for obstacle detection
    } else {
        // Safe zone, allow BB1 to explore and move freely
        Serial.println("Moderate space, continuing cautious approach...");
        cautiousApproach(); // Proceed with caution
    }

    // Adjust behavior based on score and environment
    if (score < 0) {
        Serial.println("Low score, applying extra caution...");
        cautiousApproach(); // Extra cautious approach for low score
    } else {
        Serial.println("High score, exploring freely...");
        idleWander(); // Continue casual wandering
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
    int frontDistance = getUltrasonicDistance();  // Front distance
    int IR_Left_Value = digitalRead(IR_LEFT);  // Left sensor
    int IR_Right_Value = digitalRead(IR_RIGHT);  // Right sensor

    if (frontDistance > 100) {  // If there's a lot of space ahead
        moveForward();  // Safe to move forward
        recordPath(frontDistance, "moveForward");  // Record the path
        delay(1000);  // Move forward for a bit
    } else if (frontDistance < 30) {  // Too close to something, change direction
        reactToCloseObstacle();  // Handle the obstacle appropriately
        recordPath(frontDistance, "spinLeft");  // Record the action
    }

    // Occasionally spin to get a broader view of the environment
    if (random(0, 10) < 2) {  // 20% chance to spin left
        spinLeft();
        recordPath(frontDistance, "spinLeft");
        delay(1000);
    } else if (random(0, 10) < 2) {  // 20% chance to spin right
        spinRight();
        recordPath(frontDistance, "spinRight");
        delay(1000);
    }

    // Update the environment map with current position
    currentPosition++;
    updateMap();  // Update the environment map with new data
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
    -webkit-touch-callout: none;  /* Disable text selection on touchscreens */
    -webkit-user-select: none;  /* Disable text selection on touch devices */
    -moz-user-select: none;  /* Disable text selection on Mozilla browsers */
    -ms-user-select: none;  /* Disable text selection on Microsoft browsers */
    user-select: none;  /* Disable text selection for all browsers */
  }
  h1 {
    font-size: 24px; 
    margin: 0;  /* No extra margin */
  }
  .button { 
    border: none; 
    border-radius: 12px; 
    padding: 20px 40px; 
    font-size: 20px; 
    color: #fff; 
    cursor: pointer; 
    outline: none; 
    margin: 10px; 
    -webkit-user-select: none;  /* Ensure no text selection on buttons */
    user-select: none;  /* Prevent text selection for all buttons */
  }
  .button:focus { 
    outline: none;  /* No focus outline */
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
    gap: 10px; 
  }
  @media (max-width: 768px) {  
    .button {
      font-size: 16px; 
      padding: 15px 30px; 
    }
    h1 {
      font-size: 20px; 
    }
  }
</style>
</head>
<body>
<h1>Robot Control Interface</h1>
<div id="controlGrid">
  <button class="button special" id="autoButton" onclick='sendCommand("/auto"); return false;'>Auto</button>
  <button class="button special" id="exploreButton" onclick='sendCommand("/explore"); return false;'>Explore</button>
  <button class="button control" id="forwardButton" ontouchstart='sendCommand("/forward"); return false;' ontouchend='sendCommand("/stop"); return false;'>Forward</button>
  <button class="button control" id="leftButton" ontouchstart='sendCommand("/left"); return false;' ontouchend='sendCommand("/stop"); return false;'>Left</button>
  <button class="button stop" id="stopButton" ontouchstart='sendCommand("/stop"); return false;'>Stop</button>
  <button class="button control" id="rightButton" ontouchstart='sendCommand("/right"); return false;' ontouchend='sendCommand("/stop"); return false;'>Right</button>
  <button class="button control" id="backwardButton" ontouchstart='sendCommand("/backward"); return false;' ontouchend='sendCommand("/stop"); return false;'>Backward</button>
  <button class="button special" id="danceButton" onclick='sendCommand("/dance"); return false;'>Dance</button>
  <button class="button special" id="expressEmotionButton' onclick='sendCommand("/expressEmotion"); return false;'>Express Emotion</button>
</div>
<iframe id="logFrame" srcdoc="<p>Command log initialized...</p>" style="width: 100%; height: 20%; border: none;"></iframe> 
<script>
function sendCommand(command) {
    var xhr = new XMLHttpRequest(); // Create a new XMLHttpRequest object
    xhr.open('GET', command, true); 
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE) { 
            if (xhr.status == 200) { 
                var logFrame = document.getElementById("logFrame");
                var logDocument = logFrame.contentDocument || logFrame.contentWindow.document;
                logDocument.body.innerHTML = "<p>" + command + " command executed.</p>" + logDocument.body.innerHTML;  // Add new log at top
            } else {
                console.error("Failed to execute command: " + xhr.status); 
            }
        }
    };
    xhr.send(); 
    return false; 
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
    if (distance < 50) {  // Assuming 50 cm is a safe stopping distance before head impact
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





// Function to record the current path and action taken
void recordPath(int distance, String action) {
    distances.push_back(distance);
    actions.push_back(action);
    if (environmentMap.find(currentPosition) == environmentMap.end()) {
        environmentMap[currentPosition] = {distance};  // Store the sensor data
    } else {
        environmentMap[currentPosition].push_back(distance);  // Append to existing data
    }
    currentPosition++;  // Increment position
}



void updateMap() {
    int currentDistance = getUltrasonicDistance();
    recordPath(currentDistance, "moveForward");
}

void adjustBehavior() {
    if (score < 0) {
        Serial.println("Low score, applying extra caution...");
        cautiousApproach();  // Extra cautious approach for low score
    } else {
        Serial.println("High score, exploring freely...");
        idleWander();  // Continue casual wandering
    }
}

void calculateScore(bool avoidedObstacle) {
    if (avoidedObstacle) {
        score += 10;  // Reward for avoiding obstacle
    } else {
        score -= 10;  // Penalty for hitting obstacle
    }

    adjustBehavior();  // Adjust behavior based on the updated score
}
void adjustBehaviorBasedOnScore() {
    if (score < -10) {
        Serial.println("Score very low, reverting to idle wander...");
        idleWander();  // Revert to idle wander when score is low
    } else if (score >= 0 && score < 10) {
        Serial.println("Moderate score, cautious approach...");
        cautiousApproach();  // Apply cautious approach when score is moderate
    } else if (score >= 10) {
        Serial.println("High score, exploring freely...");
        idleWander();  // Allow free wandering for high scores
    }
}

// Function for navigating
void navigate() {
    int currentDistance = getUltrasonicDistance();

    if (currentPosition > 0 && environmentMap[currentPosition - 1][0] < 30) {
        spinRight();  // Adjust behavior based on learned environment
        delay(500);
    } else if (currentDistance < 30) {
        reactToCloseObstacle();  // Default behavior when near obstacle
    } else {
        moveForward();  // Clear path, move forward
    }
}
void manageMemory() {
    const int maxMapSize = 100;  // Maximum size for the environment map

    // Remove old entries if the map grows too large
    if (environmentMap.size() > maxMapSize) {
        auto oldestKey = environmentMap.begin()->first;  // Get the oldest key
        environmentMap.erase(oldestKey);  // Remove the oldest entry
    }
}
#define MAX_BUFFER_SIZE 100  // Maximum size for the circular buffer

// Circular buffer for distances and actions
int distanceBuffer[MAX_BUFFER_SIZE];
String actionBuffer[MAX_BUFFER_SIZE];
int bufferIndex = 0;  // Current index in the circular buffer

void addDataToBuffer(int distance, String action) {
    // Add data to the buffer and update the index
    distanceBuffer[bufferIndex] = distance;
    actionBuffer[bufferIndex] = action;
    bufferIndex = (bufferIndex + 1) % MAX_BUFFER_SIZE;  // Wrap around if needed
}
