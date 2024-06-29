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
#define TRIG_PIN_REAR 4
#define ECHO_PIN_REAR 13

#define PIR_SENSOR_PIN 12

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

void setup() {
    Serial.begin(115200);

    // Set up Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

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

void handleExpressEmotion() {
    String body = server.arg("plain");
    DynamicJsonDocument doc(256);
    deserializeJson(doc, body);

    String emotion = doc["emotion"];

    if (emotion == "happy") {
        expressEmotion("happy");
    } else if (emotion == "sad") {
        expressEmotion("sad");
    } else if (emotion == "startled") {
        expressEmotion("startled");
    }

    server.send(200, "text/plain", "Emotion received");
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

    if (millis() - lastActionTime < 500) {
        return; // Ensure actions are non-blocking by adding delay between actions
    }

    if (!isIdle) {
        action = random(0, 100);
        if (action < 20) {
            idleStartTime = millis();
            isIdle = true;
            return;
        } else if (action < 50) {
            moveForward();
        } else if (action < 75) {
            spinLeft();
        } else {
            spinRight();
        }
        expressEmotion("happy");
        lastActionTime = millis();
    } else {
        if (millis() - idleStartTime >= random(1000, 5000)) {
            isIdle = false;
        }
    }
}

void reactToCloseObstacle() {
    stopMotors();
    moveBackward();
    delay(500);
    stopMotors();

    if (random(0, 2)) {
        spinLeft();
        delay(200);
    } else {
        spinRight();
        delay(200);
    }
    stopMotors();
}

void expressEmotion(String emotion) {
    if (emotion == "happy") {
        Serial.println("BB1 is happy!");
    } else if (emotion == "startled") {
        Serial.println("BB1 is startled!");
    }
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

void exploreEnvironmentEnhanced() {
    seekWall();
    followWall();
    navigateToCenter();
}

void seekWall() {
    Serial.println("Seeking wall...");
    while (true) {
        int distance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);
        if (distance < 60) {
            stopMotors();
            break;
        }
        moveForward();
        delay(100);
    }
}

void followWall() {
    Serial.println("Following wall using front sensor...");
    const int safeDistance = SAFE_DISTANCE;
    int currentDistance;

    while (true) {
        currentDistance = getUltrasonicDistance(TRIG_PIN_FRONT, ECHO_PIN_FRONT);

        if (currentDistance < safeDistance - ADJUST_DISTANCE) {
            spinRight();
            delay(200);
        } else if (currentDistance > safeDistance + ADJUST_DISTANCE) {
            spinLeft();
            delay(200);
        } else {
            moveForward();
            delay(100);
        }

        if (digitalRead(PIR_SENSOR_PIN) == HIGH) {
            expressEmotion("happy");
        }
    }
}

void navigateToCenter() {
    Serial.println("Navigating to center...");
    moveForward();
    delay(5000);
    stopMotors();
    Serial.println("Assumed center reached.");
}

Point calculateRoomCenter() {
    Point center = {50, 50};
    return center;
}

std::vector<Point> findPathToCenter(Point center) {
    std::vector<Point> path;
    path.push_back(center);
    return path;
}

void moveTo(Point step) {
    Serial.print("Moving to X: ");
    Serial.print(step.x);
    Serial.print(", Y: ");
    Serial.println(step.y);
    // Implement the actual movement logic here based on your coordinate system
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
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
  }
  h1 {
    font-size: 24px;
    margin: 0;
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
    -webkit-user-select: none;
    user-select: none;
  }
  .button:focus {
    outline: none;
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
  <button class="button special" id="expressEmotionButton" onclick='sendCommand("/expressEmotion"); return false;'>Express Emotion</button>
  <button class="button special" id="returnHomeButton" onclick='sendCommand("/return_home"); return false;'>Return Home</button>
</div>
<iframe id="logFrame" srcdoc="<p>Command log initialized...</p>" style="width: 100%; height: 20%; border: none;"></iframe>
<script>
function sendCommand(command) {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', command, true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == XMLHttpRequest.DONE) {
            if (xhr.status == 200) {
                var logFrame = document.getElementById("logFrame");
                var logDocument = logFrame.contentDocument || logFrame.contentWindow.document;
                logDocument.body.innerHTML = "<p>" + command + " command executed.</p>" + logDocument.body.innerHTML;
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

    moveForward();
    delay(1000);
    spinLeft();
    delay(1000);
    moveForward();
    delay(1000);
    spinRight();
    delay(1000);
    stopMotors();

    for (int i = 0; i < 5; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }

    spinRight();
    delay(3000);
    stopMotors();

    Serial.println("Dance routine complete!");
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
    if (score < -10) {
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
