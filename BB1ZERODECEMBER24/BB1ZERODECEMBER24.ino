#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <vector>
#include <map>
#include <Wire.h>
#include <MPU6050.h>
#include <stack>

// Point structure for tracking coordinates
struct Point {
    int x, y;
};

// Global declarations
std::vector<int> distances;
std::vector<String> actions;
std::stack<Point> pathStack;
std::stack<Point> movementStack;

int currentPosition = 0;
int score = 0;
std::map<int, std::vector<int>> environmentMap;
const char* ssid = "BB1";
const char* password = "totallysecure";

#define IN1_LEFT 19
#define IN2_LEFT 15
#define IN1_RIGHT 5
#define IN2_RIGHT 18

#define TRIG_PIN_FRONT 2
#define ECHO_PIN_FRONT 17
#define TRIG_PIN_REAR 16
#define ECHO_PIN_REAR 13

#define PIR_SENSOR_PIN 12
#define SWEEP_STEP_ANGLE 15 // Degrees per step
#define SWEEP_DELAY 200     // Time to pause between steps (in ms)
int sweepBuffer[360 / SWEEP_STEP_ANGLE]; // To hold distances for each step

MPU6050 mpu;
int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile bool isManualControl = true;
bool isStuck = false;
long duration;
int distance;

unsigned long startTime = 0;
unsigned long lastActionTime = 0;

volatile int leftPulseCount = 0;
volatile int rightPulseCount = 0;
int lastLeftPulseCount = 0;
int lastRightPulseCount = 0;
int speedLeft = 255;
int speedRight = 255;
volatile int leftHallPulseCount = 0;
volatile int rightHallPulseCount = 0;

WebServer server(80);

// Switch smoothing variables
int switch1;
float switch1Smoothed;
float switch1Prev;

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

// Movement functions
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void stopMotors();
void danceRoutine();
void exploreEnvironment();
void autoMove();
void cautiousApproach();
void idleWander();
void reactToCloseObstacle();
void expressEmotion(String emotion);
void adjustBehaviorBasedOnScore();
void calculateScore(bool avoidedObstacle);
void updateMap();
int getUltrasonicDistance(int trigPin, int echoPin);
void navigate();
void manageMemory();
void navigateBasedOnDistance(int frontDist, int rearDist);
void moveTo(Point step);
void returnToStart();
void turnAround();
void handleFrontObstacle();
void handleRearObstacle();
void handleExpressEmotion();
void performTaskA();
void performTaskB();
void performTaskC();

void recordPath(int distance, String action) {
    distances.push_back(distance);
    actions.push_back(action);
    if (environmentMap.find(currentPosition) == environmentMap.end()) {
        environmentMap[currentPosition] = {distance};
    } else {
        environmentMap[currentPosition].push_back(distance);
    }
    currentPosition++;
}

int getUltrasonicDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    int distance = duration * 0.034 / 2;
    Serial.print("Distance measured on pin ");
    Serial.print(echoPin);
    Serial.print(": ");
    Serial.println(distance);
    return distance;
}

#define SAFE_DISTANCE 40 // Increased safe distance
#define ADJUST_DISTANCE 10

void navigateBasedOnDistance(int frontDist, int rearDist) {
    if (frontDist < SAFE_DISTANCE) {
        handleFrontObstacle();
    } else if (rearDist < SAFE_DISTANCE) {
        handleRearObstacle();
    } else {
        moveForward();
    }
}

void handleFrontObstacle() {
    stopMotors();
    moveBackward();
    delay(500); // Move back a bit
    spinRight(); // Try to turn away from the obstacle (changed to spinRight)
    delay(200);
    stopMotors();
}

void handleRearObstacle() {
    stopMotors();
    moveForward();
    delay(500); // Move forward a bit
    spinLeft(); // Try to turn away from the obstacle (changed to spinLeft)
    delay(200);
    stopMotors();
}

int frontDistance, rearDistance;
// Declare and define the HTTP handler
void handleExpressEmotion() {
    if (server.hasArg("emotion")) {
        String emotion = server.arg("emotion");
        expressEmotion(emotion);
        server.send(200, "text/plain", "Emotion expressed: " + emotion);
    } else {
        server.send(400, "text/plain", "Error: No emotion specified.");
    }
}

// Define the expressEmotion function
void expressEmotion(String emotion) {
    if (emotion == "happy") {
        Serial.println("BB1 is happy!");
    } else if (emotion == "startled") {
        Serial.println("BB1 is startled!");
    } else if (emotion == "mad") {
        Serial.println("BB1 is mad!");
    } else if (emotion == "sad") {
        Serial.println("BB1 is sad.");
    } else if (emotion == "bored") {
        Serial.println("BB1 is bored...");
    } else {
        Serial.println("Unknown emotion.");
    }
}

void setup() {
    Serial.begin(115200);

    // Connect to Wi-Fi
    WiFi.begin("INADAZE", "jeansrocket543");
    Serial.print("Connecting to Wi-Fi");
    
    // Wait for connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Initialize sensors and motors
    frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    rearDistance = getUltrasonicDistance(TRIG_PIN_REAR, ECHO_PIN_REAR);
    Wire.begin();
    mpu.initialize();
    pinMode(PIR_SENSOR_PIN, INPUT);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
    } else {
        Serial.println("MPU6050 connection successful.");
    }

    startTime = millis();

    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);
    pinMode(TRIG_PIN_FRONT, OUTPUT);
    pinMode(ECHO_PIN_FRONT, INPUT);
    pinMode(TRIG_PIN_REAR, OUTPUT);
    pinMode(ECHO_PIN_REAR, INPUT);

    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(32, INPUT_PULLUP);
    pinMode(33, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(34), onLeftHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(35), onLeftHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(32), onRightHallSensor, RISING);
    attachInterrupt(digitalPinToInterrupt(33), onRightHallSensor, RISING);

    server.on("/", HTTP_GET, []() {
        server.send(200, "text/html", controlPage());
    });

    server.on("/forward", HTTP_GET, []() {
        isManualControl = true;
        moveForward();
        server.send(200, "text/plain", "Moving forward");
    });

    server.on("/backward", HTTP_GET, []() {
        isManualControl = true;
        moveBackward();
        server.send(200, "text/plain", "Moving backward");
    });

    server.on("/left", HTTP_GET, []() {
        isManualControl = true;
        spinLeft();
        server.send(200, "text/plain", "Turning left");
    });

    server.on("/right", HTTP_GET, []() {
        isManualControl = true;
        spinRight();
        server.send(200, "text/plain", "Turning right");
    });

    server.on("/stop", HTTP_GET, []() {
        isManualControl = true;
        stopMotors();
        server.send(200, "text/plain", "Stopping");
    });

    server.on("/auto", HTTP_GET, []() {
        isManualControl = false;
        server.send(200, "text/plain", "Switched to auto mode");
    });

    server.on("/explore", HTTP_GET, []() {
        isManualControl = false;
        exploreEnvironment();
        server.send(200, "text/plain", "Exploration mode activated");
    });

    server.on("/dance", HTTP_GET, []() {
        danceRoutine();
        server.send(200, "text/plain", "Dance sequence activated");
    });

    server.on("/return_home", HTTP_GET, []() {
        returnToStart();
        server.send(200, "text/plain", "Returning to start position");
    });

    server.on("/task_a", HTTP_GET, []() {
        performTaskA();
        server.send(200, "text/plain", "Task A activated");
    });

    server.on("/task_b", HTTP_GET, []() {
        performTaskB();
        server.send(200, "text/plain", "Task B activated");
    });

    server.on("/task_c", HTTP_GET, []() {
        performTaskC();
        server.send(200, "text/plain", "Task C activated");
    });

    server.on("/start_tracking", HTTP_GET, []() {
        Serial.println("Tracking mode activated.");
        server.send(200, "text/plain", "Tracking mode activated");
    });

    server.on("/human_detected", HTTP_GET, []() {
        int pirState = digitalRead(PIR_SENSOR_PIN);
        String response = pirState == HIGH ? "Human detected" : "No human detected";
        server.send(200, "text/plain", response);
    });

    server.on("/hall_sensors", HTTP_GET, []() {
        StaticJsonDocument<200> doc;
        doc["left_hall"] = leftHallPulseCount;
        doc["right_hall"] = rightHallPulseCount;
        String output;
        serializeJson(doc, output);
        server.send(200, "application/json", output);
    });

    server.on("/expressEmotion", HTTP_GET, []() {
        expressEmotion("happy");
        server.send(200, "text/plain", "Emotion expressed");
    });

    server.on("/sensors", HTTP_GET, []() {
        int frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        int rearDistance = getUltrasonicDistance(TRIG_PIN_REAR, ECHO_PIN_REAR);

        StaticJsonDocument<256> doc;
        doc["front_distance"] = frontDistance;
        doc["rear_distance"] = rearDistance;

        String sensorData;
        serializeJson(doc, sensorData);
        server.send(200, "application/json", sensorData);
    });

    server.on("/gyro", HTTP_GET, []() {
        String gyroData = getGyroData();
        server.send(200, "application/json", gyroData);
    });

    server.on("/getData", HTTP_GET, []() {
        String data = "{ \"distances\": [";
        for (int d : distances) {
            data += String(d) + ",";
        }
        data.remove(data.length() - 1);
        data += "], \"actions\": [";
        for (String a : actions) {
            data += "\"" + a + "\",";
        }
        data.remove(data.length() - 1);
        data += "] }";

        server.send(200, "application/json", data);
    });

server.on("/expressEmotion", HTTP_POST, handleExpressEmotion);


    server.begin();
    Serial.println("HTTP server started. Ready for commands.");
}

String getGyroData() {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    StaticJsonDocument<200> doc;
    doc["gx"] = gx;
    doc["gy"] = gy;
    doc["gz"] = gz;

    String gyroData;
    serializeJson(doc, gyroData);
    return gyroData;
}

void smoothStartMotors() {
    static unsigned long lastStepTime = 0;
    static int speed = 0;
    const int targetSpeed = 255;
    const int increment = 10;
    unsigned long currentMillis = millis();

    if (currentMillis - lastStepTime >= 20) { // Update every 20 milliseconds
        lastStepTime = currentMillis;

        if (speed <= targetSpeed) {
            digitalWrite(IN1_LEFT, speed < targetSpeed ? HIGH : LOW);
            digitalWrite(IN1_RIGHT, speed < targetSpeed ? HIGH : LOW);
            speed += increment;
        }
    }
}

void checkIfStuck() {
    static unsigned long lastCheckTime = 0;
    const long checkInterval = 1000;

    if (millis() - lastCheckTime >= checkInterval) {
        if (lastLeftPulseCount == leftHallPulseCount && lastRightPulseCount == rightHallPulseCount) {
            if (!isStuck) {
                Serial.println("BB1 might be stuck, stopping motors...");
                stopMotors();
                isStuck = true;
            }
        } else {
            lastLeftPulseCount = leftHallPulseCount;
            lastRightPulseCount = rightHallPulseCount;
            isStuck = false;
        }
        lastCheckTime = millis();
    }
}
void sonarSweep() {
    Serial.println("Starting sonar sweep...");
    int totalSteps = 360 / SWEEP_STEP_ANGLE;
    int bestDirection = -1;
    int maxDistance = 0;

    // Perform the sweep
    for (int i = 0; i < totalSteps; i++) {
        // Get distance reading
        int distance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        sweepBuffer[i] = distance;

        // Log the reading
        Serial.print("Step ");
        Serial.print(i);
        Serial.print(": Distance = ");
        Serial.println(distance);

        // Track the best (most open) direction
        if (distance > maxDistance) {
            maxDistance = distance;
            bestDirection = i;
        }

        // Rotate for the next reading
        spinRight();
        delay(SWEEP_DELAY);
        stopMotors();
    }

    // Return to the best direction
    if (bestDirection >= 0) {
        int angleToBestDirection = bestDirection * SWEEP_STEP_ANGLE;
        Serial.print("Best direction found at ");
        Serial.print(angleToBestDirection);
        Serial.println(" degrees.");
        
        // Turn toward the best direction
        if (angleToBestDirection <= 180) {
            spinRight();
        } else {
            spinLeft();
            angleToBestDirection = 360 - angleToBestDirection; // Adjust for left turn
        }
        delay(angleToBestDirection * (SWEEP_DELAY / SWEEP_STEP_ANGLE));
        stopMotors();
    } else {
        Serial.println("No clear path detected. Staying put.");
    }

    Serial.println("Sonar sweep complete.");
}

void loop() {
    server.handleClient();
    checkIfStuck();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Adjust for the sensor being rotated 90 degrees counterclockwise
    int16_t adjusted_ax = ay;  // X-axis is now Y-axis
    int16_t adjusted_ay = -ax; // Y-axis is now -X-axis
    int16_t adjusted_gx = gy;  // Gyro X-axis is now Gyro Y-axis
    int16_t adjusted_gy = -gx; // Gyro Y-axis is now -Gyro X-axis

    // Use adjusted_ax, adjusted_ay, adjusted_gx, adjusted_gy instead of ax, ay, gx, gy in your logic
    if (abs(adjusted_ax) > 15000 || abs(adjusted_ay) > 15000 || abs(az) > 15000) {
        Serial.println("Significant acceleration detected! Stopping motors.");
        stopMotors();
        return;
    }

    if (abs(adjusted_gx) > 15000 || abs(adjusted_gy) > 15000 || abs(gz) > 15000) {
        Serial.println("Significant angular rate change detected! Stopping motors.");
        stopMotors();
        return;
    }

    // Update distances
    int newFrontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    int newRearDistance = getUltrasonicDistance(TRIG_PIN_REAR, ECHO_PIN_REAR);

    if (newFrontDistance != frontDistance || newRearDistance != rearDistance) {
        frontDistance = newFrontDistance;
        rearDistance = newRearDistance;
        Serial.print("Front: ");
        Serial.print(frontDistance);
        Serial.print(" cm, Rear: ");
        Serial.print(rearDistance);
        Serial.println(" cm");
    }

    // *** Switch smoothing logic ***
    switch1 = digitalRead(12); // read switch
    switch1 = switch1 * 100;   // multiply by 100

    // Smoothing
    switch1Smoothed = (switch1 * 0.05) + (switch1Prev * 0.95);
    switch1Prev = switch1Smoothed;
    // *** End of switch smoothing logic ***

    Serial.print(switch1);              // print to serial terminal/plotter
    Serial.print(" ");
    Serial.println(switch1Smoothed);

    if (isManualControl) {
        // Manual control mode, handle HTTP requests
    } else {
        // Automatic control mode
        if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
            expressEmotion("happy");
            if (frontDistance > 100) {
                moveForward();
            } else if (frontDistance < 30) {
                stopMotors();
                reactToCloseObstacle();
            }
            updateMap();
        } else {
            autoMove();
        }

        static unsigned long lastReportTime = 0;
        const unsigned long reportInterval = 1000;
        if (millis() - lastReportTime >= reportInterval) {
            lastReportTime = millis();
            Serial.print("Periodic Distance Report - Front: ");
            Serial.print(frontDistance);
            Serial.print(" cm, Rear: ");
            Serial.println(rearDistance);
            recordPath(frontDistance, "sensor update");
            updateMap();
        }

        navigateBasedOnDistance(frontDistance, rearDistance);
    }
}

void triggerEmotion(String emotion) {
    HTTPClient http;
    const char* listenerIP = "192.168.1.3";
    String endpoint = "/expressEmotion";
    String serverUrl = "http://" + String(listenerIP) + endpoint;

    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");

    DynamicJsonDocument doc(256);
    doc["emotion"] = emotion;

    String jsonData;
    serializeJson(doc, jsonData);

    int httpCode = http.POST(jsonData);

    if (httpCode == 200) {
        Serial.println("Emotion triggered successfully.");
    } else {
        Serial.print("Error triggering emotion: HTTP code ");
        Serial.println(httpCode);
    }

    http.end();
}


void autoMove() {
    adjustBehaviorBasedOnScore();
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (abs(ax) > 15000 || abs(ay) > 15000 || abs(az) > 15000) {
        Serial.println("Significant acceleration detected! Stopping motors.");
        stopMotors();
        return;
    }

    if (abs(gx) > 15000 || abs(gy) > 15000 || abs(gz) > 15000) {
        Serial.println("Significant angular rate change detected! Stopping motors.");
        stopMotors();
        return;
    }

    int distance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

    if (distance > 100) {
        Serial.println("Open space, initiating idle wander...");
        idleWander();
    } else if (distance < 30) {
        Serial.println("Obstacle detected, initiating cautious approach...");
        reactToCloseObstacle();
        calculateScore(false);
    } else {
        Serial.println("Moderate space, continuing cautious approach...");
        cautiousApproach();
    }

    if (score < 0) {
        Serial.println("Low score, applying extra caution...");
        cautiousApproach();
    } else {
        Serial.println("High score, exploring freely...");
        idleWander();
    }
}

unsigned long idleStartTime = 0;
bool isIdle = false;

void idleWander() {
    static int action = -1;
    static unsigned long idleStartTime = 0;
    static bool isIdle = false;

    if (millis() - lastActionTime < 500) {
        return; // Ensure actions are non-blocking by adding delay between actions
    }

    if (!isIdle) {
        action = random(0, 100); // Choose a random action

        if (action < 20) { // Do nothing, idle for a random duration
            idleStartTime = millis();
            isIdle = true;
            Serial.println("BB1 is chilling...");
            return;
        } else if (action < 50) { // Move forward
            int dist = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
            if (dist > SAFE_DISTANCE) {
                moveForward();
                Serial.println("BB1 is wandering forward.");
            } else {
                Serial.println("Too close to wall during idle wander, adjusting...");
                spinRight(); // Turn away from obstacle
                delay(500);
            }
        } else if (action < 75) { // Spin left
            spinLeft();
            Serial.println("BB1 is spinning left.");
        } else { // Spin right
            spinRight();
            Serial.println("BB1 is spinning right.");
        }

        expressEmotion("happy"); // Make BB1 show happiness during idle wander
        lastActionTime = millis(); // Update the action timestamp
    } else {
        // Idle duration completed
        if (millis() - idleStartTime >= random(1000, 5000)) {
            isIdle = false; // Stop idling and pick a new action
            Serial.println("BB1 is done chilling.");
        }
    }
}

void reactToCloseObstacle() {
    stopMotors();
    Serial.println("Obstacle detected. Initiating sonar sweep...");
    sonarSweep(); // Use the sweep to find an open path
    moveForward(); // Move forward in the chosen direction
}



void cautiousApproach() {
    int distance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    if (distance > SAFE_DISTANCE) {
        moveForward();
        expressEmotion("happy");
    } else if (distance < SAFE_DISTANCE) {
        reactToCloseObstacle();
        expressEmotion("startled");
    } else {
        idleWander();
    }
}

void adjustRight(int power) {
    analogWrite(IN1_LEFT, 255);
    analogWrite(IN1_RIGHT, 255 - power);
}

void adjustLeft(int power) {
    analogWrite(IN1_LEFT, 255 - power);
    analogWrite(IN1_RIGHT, 255);
}

void returnToStart() {
    Serial.println("Returning to start...");
    turnAround();
    while (!movementStack.empty()) {
        Point step = movementStack.top();
        movementStack.pop();
        moveTo(step);
        delay(1000);
    }
    Serial.println("Returned to start position.");
}

void turnAround() {
    // Assuming you want to turn 180 degrees
    const int targetAngle = 180;
    int16_t initialGyroZ;
    mpu.getRotation(&gx, &gy, &initialGyroZ);
    int16_t currentGyroZ = initialGyroZ;
    int16_t totalTurned = 0;

    // Get initial distances to front and rear
    int frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    int rearDistance = getUltrasonicDistance(TRIG_PIN_REAR, ECHO_PIN_REAR);

    bool spinLeftDirection = frontDistance > rearDistance;

    if (spinLeftDirection) {
        spinLeft();
        Serial.println("Spinning left to turn around.");
    } else {
        spinRight();
        Serial.println("Spinning right to turn around.");
    }

    while (abs(totalTurned) < targetAngle) {
        mpu.getRotation(&gx, &gy, &currentGyroZ);
        // Calculate the change in angle since the last reading
        int16_t deltaAngle = (currentGyroZ - initialGyroZ) / 131; // Gyro sensitivity for 250 dps
        if (!spinLeftDirection) {
            deltaAngle = -deltaAngle; // Reverse the angle calculation for right turn
        }
        totalTurned += deltaAngle;
        initialGyroZ = currentGyroZ;
    }

    // Stop turning
    stopMotors();
    Serial.println("Turned around 180 degrees.");
}

void exploreEnvironment() {
    Serial.println("Exploration mode activated.");
    bool exploring = true;

    unsigned long explorationStartTime = millis();

    while (exploring) {
        // Check for control commands
        server.handleClient();

        int frontDist = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        int rearDist = getUltrasonicDistance(TRIG_PIN_REAR, ECHO_PIN_REAR);

        if (frontDist < SAFE_DISTANCE) {
            handleFrontObstacle();
        } else if (rearDist < SAFE_DISTANCE) {
            handleRearObstacle();
        } else {
            moveForward();
        }

        // Break out of exploration if it takes too long
        if (millis() - explorationStartTime > 30000) { // Limit exploration time to 30 seconds
            exploring = false;
            Serial.println("Exploration mode ended due to timeout.");
        }

        // Allow stopping exploration through control panel
        if (isManualControl) {
            exploring = false;
            Serial.println("Exploration mode ended by user.");
        }
    }

    returnToStart();
}

void moveTo(Point step) {
    Serial.print("Moving to X: ");
    Serial.print(step.x);
    Serial.print(", Y: ");
    Serial.println(step.y);
    // Implement the actual movement logic here based on your coordinate system
}

void performTaskA() {
    static unsigned long taskStartTime = 0;
    static int step = 0;
    const unsigned long moveDuration = 5000; // Move for 5 seconds per side
    const unsigned long pauseDuration = 1000; // Pause for 1 second at each corner
    const int totalSteps = 4; // 4 sides of the box
    static bool obstacleDetected = false;
    static unsigned long obstacleAvoidanceStartTime = 0;
    const unsigned long obstacleAvoidanceDuration = 2000; // 2 seconds to try avoiding obstacle

    unsigned long currentTime = millis();

    if (!obstacleDetected) {
        // Check for obstacles
        int frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

        if (frontDistance < SAFE_DISTANCE) {
            obstacleDetected = true;
            obstacleAvoidanceStartTime = currentTime;
            stopMotors();
            Serial.println("Obstacle detected! Initiating avoidance maneuvers...");
            return;
        }

        // Perform the patrol task
        if (currentTime - taskStartTime < moveDuration) {
            moveForward();
        } else {
            stopMotors();
            if (currentTime - taskStartTime < moveDuration + pauseDuration) {
                // Pausing at the corner
                Serial.println("Pausing to inspect plants...");
            } else {
                // Move to the next step
                step = (step + 1) % totalSteps;
                taskStartTime = currentTime;

                if (step == 0) {
                    Serial.println("Completed a lap, reversing direction...");
                } else {
                    spinRight();
                    delay(500); // Adjust this delay for a 90-degree turn
                    stopMotors();
                }
            }
        }
    } else {
        // Obstacle avoidance logic
        if (currentTime - obstacleAvoidanceStartTime < obstacleAvoidanceDuration) {
            int frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

            if (frontDistance >= SAFE_DISTANCE) {
                obstacleDetected = false;
                Serial.println("Obstacle cleared. Resuming patrol...");
                taskStartTime = currentTime;
                return;
            } else {
                moveBackward();
                delay(500);
                stopMotors();
                spinRight();
                delay(500);
                stopMotors();
            }
        } else {
            obstacleDetected = false;
            taskStartTime = currentTime;
            Serial.println("Failed to clear obstacle within time limit. Resuming patrol...");
        }
    }
}


void performTaskB() {
    static unsigned long lastMoveTime = 0;
    static bool movingForward = true;
    const unsigned long moveInterval = 5000; // Move for 5 seconds

    unsigned long currentTime = millis();

    // Check for obstacles
    int frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

    if (frontDistance < SAFE_DISTANCE) {
        // Obstacle detected, perform an about-face
        Serial.println("Obstacle detected! Performing about-face...");
        if (movingForward) {
            moveBackward();
        } else {
            moveForward();
        }
        delay(1000); // Move back for 1 second
        stopMotors();
        spinRight();
        delay(500); // Adjust this delay for a 180-degree turn
        stopMotors();
        movingForward = !movingForward;
        lastMoveTime = currentTime;
    } else {
        // Continue patrolling
        if (currentTime - lastMoveTime >= moveInterval) {
            lastMoveTime = currentTime;
            if (movingForward) {
                moveForward();
            } else {
                moveBackward();
            }
        }
    }
}

void performTaskC() {
    static unsigned long lastMoveTime = 0;
    static unsigned long lastTurnTime = 0;
    static unsigned long chillStartTime = 0;
    static bool isChilling = false;
    const unsigned long moveInterval = 10000; // Move every 10 seconds
    const unsigned long turnInterval = 2000;  // Turn every 2 seconds during chill
    const unsigned long chillDuration = 30000; // Chill for 30 seconds

    unsigned long currentTime = millis();

    if (isChilling) {
        if (currentTime - chillStartTime >= chillDuration) {
            isChilling = false;
            Serial.println("Done chilling, moving again...");
        } else {
            // Perform turn maneuvers to face warm bodies
            if (currentTime - lastTurnTime >= turnInterval) {
                lastTurnTime = currentTime;
                if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
                    Serial.println("Warm body detected! Facing it...");
                    stopMotors();
                } else {
                    Serial.println("No warm body detected, turning...");
                    spinRight();
                    delay(200);
                    stopMotors();
                }
            }
            return;
        }
    }

    if (currentTime - lastMoveTime >= moveInterval) {
        lastMoveTime = currentTime;
        isChilling = true;
        chillStartTime = currentTime;
        Serial.println("Moving to a new position...");
        moveForward();
        delay(1000); // Move forward for 1 second
        stopMotors();
    }
}


String controlPage() {
    String html = R"(
<html>
<head>
<title>BB1ZERO MOBILE UNIT</title>
<style>
  body, html {
    margin: 0;
    padding: 0;
    width: 100%;
    height: 100%;
    display: flex;
    justify-content: center;
    align-items: center;
    overflow: hidden;
    font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
    background-color: #0f0f0f;
    color: #e0e0e0;
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
  }

  .container {
    display: flex;
    width: 100%;
    height: 100%;
    padding: 20px;
    box-sizing: border-box;
  }

  .left, .center, .right {
    flex: 1;
    display: flex;
    justify-content: center;
    align-items: center;
    padding: 20px;
    box-sizing: border-box;
  }

  .controls {
    display: grid;
    grid-template-areas: 
      "up up up"
      "left stop right"
      "down down down";
    gap: 10px;
  }

  .controls button {
    padding: 15px;
    background-color: #1a1a1a;
    color: #e0e0e0;
    border: none;
    border-radius: 10px;
    cursor: pointer;
    font-size: 16px;
    box-shadow: 5px 5px 10px #0a0a0a, -5px -5px 10px #2a2a2a;
    position: relative;
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
  }

  .controls button::before {
    content: '';
    position: absolute;
    top: -5px;
    left: -5px;
    right: -5px;
    bottom: 0;
    border-radius: 10px;
    background: linear-gradient(180deg, rgba(0, 255, 0, 0.3) 0%, rgba(0, 255, 0, 0) 70%);
    box-shadow: 0 0 10px rgba(0, 255, 0, 0.3);
  }

  .controls button:hover {
    background-color: #333;
  }

  .status-log {
    flex: 2;
    background-color: #1a1a1a;
    padding: 20px;
    box-shadow: 10px 10px 20px #0a0a0a, -10px -10px 20px #2a2a2a;
    position: relative;
    overflow: hidden;
    text-align: left;
  }

  .status-log::before {
    content: '';
    position: absolute;
    top: -10px;
    left: -10px;
    right: -10px;
    bottom: 0;
    border-radius: 10px;
    background: linear-gradient(180deg, rgba(0, 255, 0, 0.3) 0%, rgba(0, 255, 0, 0) 70%);
    box-shadow: 0 0 20px rgba(0, 255, 0, 0.3);
  }

  .buttons {
    flex: 1;
    display: grid;
    grid-template-columns: repeat(2, 1fr);
    gap: 20px;
  }

  .buttons button {
    padding: 15px;
    background-color: #1a1a1a;
    color: #e0e0e0;
    border: none;
    border-radius: 10px;
    cursor: pointer;
    font-size: 16px;
    box-shadow: 5px 5px 10px #0a0a0a, -5px -5px 10px #2a2a2a;
    position: relative;
    -webkit-user-select: none; /* Safari */
    -ms-user-select: none; /* IE 10 and IE 11 */
    user-select: none; /* Standard syntax */
  }

  .buttons button::before {
    content: '';
    position: absolute;
    top: -5px;
    left: -5px;
    right: -5px;
    bottom: 0;
    border-radius: 10px;
    background: linear-gradient(180deg, rgba(0, 255, 0, 0.3) 0%, rgba(0, 255, 0, 0) 70%);
    box-shadow: 0 0 10px rgba(0, 255, 0, 0.3);
  }

  .buttons button:hover {
    background-color: #333;
  }

  @media (max-width: 768px) {
    .buttons button {
      padding: 10px;
      font-size: 14px;
    }
  }
</style>
</head>
<body>
<div class="container">
  <div class="left">
    <div class="controls">
      <button data-command="/forward" onmousedown='startCommand("/forward")' onmouseup='stopCommand()' ontouchstart='startCommand("/forward")' ontouchend='stopCommand()' style="grid-area: up;">UP</button>
      <button data-command="/left" onmousedown='startCommand("/left")' onmouseup='stopCommand()' ontouchstart='startCommand("/left")' ontouchend='stopCommand()' style="grid-area: left;">LEFT</button>
      <button data-command="/stop" onmousedown='stopCommand()' style="grid-area: stop;">STOP</button>
      <button data-command="/right" onmousedown='startCommand("/right")' onmouseup='stopCommand()' ontouchstart='startCommand("/right")' ontouchend='stopCommand()' style="grid-area: right;">RIGHT</button>
      <button data-command="/backward" onmousedown='startCommand("/backward")' onmouseup='stopCommand()' ontouchstart='startCommand("/backward")' ontouchend='stopCommand()' style="grid-area: down;">DOWN</button>
    </div>
  </div>
  <div class="center">
    <div class="status-log" id="status-log">
      <p>Command log initialized...</p>
    </div>
  </div>
  <div class="right">
    <div class="buttons">
      <button onclick='sendCommand("/auto")'>AUTO</button>
      <button onclick='sendCommand("/explore")'>EXPLORE</button>
      <button onclick='sendCommand("/dance")'>DANCE</button>
      <button onclick='sendCommand("/expressEmotion")'>EMOTION</button>
      <button onclick='sendCommand("/task_a")'>TASK A</button>
      <button onclick='sendCommand("/task_b")'>TASK B</button>
      <button onclick='sendCommand("/task_c")'>TASK C</button>
    </div>
  </div>
</div>
<script>
let currentCommand = null;

function sendCommand(command) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', command, true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE) {
            if (xhr.status == 200) {
                var logFrame = document.getElementById("status-log");
                logFrame.innerHTML = "<p>" + command + " command executed.</p>" + logFrame.innerHTML;
            } else {
                console.error("Failed to execute command: " + xhr.status);
            }
        }
    };
    xhr.send();
}

function startCommand(command) {
    if (currentCommand !== command) {
        stopCommand();  // Ensure any previous command is stopped
        currentCommand = command;
        sendCommand(command);
    }
}

function stopCommand() {
    if (currentCommand) {
        sendCommand("/stop");
        currentCommand = null;
    }
}

document.querySelectorAll('.controls button').forEach(button => {
    button.addEventListener('mousedown', function() {
        startCommand(this.getAttribute('data-command'));
    });
    button.addEventListener('mouseup', function() {
        stopCommand();
    });
    button.addEventListener('touchstart', function() {
        startCommand(this.getAttribute('data-command'));
    });
    button.addEventListener('touchend', function() {
        stopCommand();
    });
    button.addEventListener('mouseleave', function() {
        stopCommand();
    });
});

</script>
</body>
</html>
)";
    return html;
}


void moveForward() {
    Serial.println("Moving forward...");
    digitalWrite(IN1_LEFT, HIGH);
    digitalWrite(IN2_LEFT, LOW);
    digitalWrite(IN1_RIGHT, HIGH);
    digitalWrite(IN2_RIGHT, LOW);
}

void moveBackward() {
    Serial.println("Checking for obstacles before moving backward...");
    int rearDistance = getUltrasonicDistance(TRIG_PIN_REAR, ECHO_PIN_REAR);

    if (rearDistance < 30) {
        Serial.println("Obstacle detected at rear, stopping...");
        stopMotors();
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
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, HIGH);
    digitalWrite(IN1_RIGHT, HIGH);
    digitalWrite(IN2_RIGHT, LOW);
}

void spinRight() {
    Serial.println("Spinning right...");
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

bool checkSpaceForDance() {
    int frontDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    Serial.print("Front distance: ");
    Serial.println(frontDistance);
    if (frontDistance < 100) {
        Serial.println("Not enough space to dance :(");
        return false;
    }
    return true;
}

void danceRoutine() {
    if (!checkSpaceForDance()) {
        Serial.println("Not enough space to dance :(");
        return;
    }

    Serial.println("Starting dance routine...");

    // Move forward and spin
    moveForward();
    delay(1000);
    spinLeft();
    delay(1000);
    moveForward();
    delay(1000);
    spinRight();
    delay(1000);

    // Jiggle
    for (int i = 0; i < 5; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }

    // Moonwalk
    moonwalk();
    delay(3000);

    // Spin and wiggle
    spinRight();
    delay(1000);
    for (int i = 0; i < 3; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }
    spinLeft();
    delay(1000);

    // More dance moves
    moveForward();
    delay(1000);
    moveBackward();
    delay(1000);
    spinRight();
    delay(1000);
    spinLeft();
    delay(1000);

    stopMotors();

    Serial.println("Dance routine complete!");
}

void moonwalk() {
    Serial.println("Moonwalking...");

    // Perform a smooth backward movement with slight left-right adjustments to mimic a moonwalk
    for (int i = 0; i < 10; i++) {
        moveBackward();
        delay(100);
        spinRight();
        delay(50);
        spinLeft();
        delay(50);
    }

    stopMotors();
    Serial.println("Moonwalk complete!");
}

void updateMap() {
    int currentDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
    recordPath(currentDistance, "moveForward");
}

void adjustBehavior() {
    if (score < 0) {
        Serial.println("Low score, applying extra caution...");
        cautiousApproach();
    } else {
        Serial.println("High score, exploring freely...");
        idleWander();
    }
}

void calculateScore(bool avoidedObstacle) {
    if (avoidedObstacle) {
        score += 10;
    } else {
        score -= 10;
    }

    adjustBehavior();
}

void adjustBehaviorBasedOnScore() {
    if (score < -50) { // Wall-Spanking AI kicks in at this threshold
        Serial.println("BB1 is grounded for reckless behavior...");
        wallSpankingTimeout();
    } else if (score < -10) {
        Serial.println("Score is very low. Robot is mad.");
        triggerEmotion("mad");
        cautiousApproach();
    } else if (score < 0) {
        Serial.println("Score is low. Robot is grumpy.");
        triggerEmotion("sad");
        cautiousApproach();
    } else if (score >= 10) {
        Serial.println("High score! Robot is happy.");
        triggerEmotion("happy");
        idleWander();
    } else {
        Serial.println("Score is moderate. Robot might be bored.");
        triggerEmotion("bored");
        cautiousApproach();
    }
}

void wallSpankingTimeout() {
    stopMotors(); // Stop all movement
    expressEmotion("startled"); // Make BB1 look surprised
    Serial.println("Wall-Spanking AI activated. Timeout for reckless behavior...");
    delay(5000); // BB1 gets a 5-second timeout
    score = 0;   // Reset the score and forgive BB1
    Serial.println("Timeout complete. Resuming normal behavior.");
}

void navigate() {
    int currentDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

    // Check for obstacles and navigate accordingly
    if (currentDistance < SAFE_DISTANCE) {
        reactToCloseObstacle();
        calculateScore(false);
    } else {
        moveForward();
        calculateScore(true);
    }

    // Additional navigation logic based on recorded paths and environment map
    if (environmentMap.find(currentPosition) != environmentMap.end()) {
        std::vector<int> distancesAtCurrentPosition = environmentMap[currentPosition];
        for (int recordedDistance : distancesAtCurrentPosition) {
            if (recordedDistance < SAFE_DISTANCE) {
                stopMotors();
                reactToCloseObstacle();
                calculateScore(false);
                return;
            }
        }
    }

    // Keep track of movement for return navigation
    Point currentStep = {currentPosition, currentDistance};
    movementStack.push(currentStep);
    currentPosition++;
}

void manageMemory() {
    const int maxMapSize = 100;

    if (environmentMap.size() > maxMapSize) {
        auto oldestKey = environmentMap.begin()->first;
        environmentMap.erase(oldestKey);
    }
}

#define MAX_BUFFER_SIZE 100

int distanceBuffer[MAX_BUFFER_SIZE];
String actionBuffer[MAX_BUFFER_SIZE];
int bufferIndex = 0;

void addDataToBuffer(int distance, String action) {
    distanceBuffer[bufferIndex] = distance;
    actionBuffer[bufferIndex] = action;
    bufferIndex = (bufferIndex + 1) % MAX_BUFFER_SIZE;
}
