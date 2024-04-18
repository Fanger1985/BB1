#include <WiFi.h>
#include <WebServer.h>

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

// Web server on port 80
WebServer server(80);

// Forward declarations of functions
void moveForward();
void moveBackward();
void spinLeft();
void spinRight();
void stopMotors();
int getUltrasonicDistance();
// Forward declarations of ISR functions
void IRAM_ATTR onLeftEncoder();
void IRAM_ATTR onRightEncoder();
// Global state variable to determine control mode
volatile bool isManualControl = false;

void setup() {
    Serial.begin(9600);
    pinMode(IN1_LEFT, OUTPUT);
    pinMode(IN2_LEFT, OUTPUT);
    pinMode(IN1_RIGHT, OUTPUT);
    pinMode(IN2_RIGHT, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(IR_LEFT, INPUT);
    pinMode(IR_RIGHT, INPUT);

    // Correct the pin numbers for your setup if needed
    attachInterrupt(digitalPinToInterrupt(IN1_LEFT), onLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(IN1_RIGHT), onRightEncoder, RISING);

    // Connect to WiFi
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi network.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    // Wait a few seconds before starting the main activities
    Serial.println("Preparing to start operations...");
    delay(5000);  // Delay for 5 seconds before starting operations

    // Setup web server routes
    server.on("/", HTTP_GET, []() {
        isManualControl = false; // Allow auto-moving when visiting the main page
        server.send(200, "text/html", controlPage()); // Send dynamic HTML content
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
        isManualControl = false; // Allow the robot to move autonomously
        server.send(200, "text/plain", "Switched to auto mode");
    });
    server.on("/explore", HTTP_GET, []() {
        isManualControl = true; // Take manual control for exploration
        exploreEnvironment();
        server.send(200, "text/plain", "Exploration mode activated");
    });

    server.on("/dance", HTTP_GET, []() {
        isManualControl = true; // Take manual control for the dance
        danceRoutine();
        server.send(200, "text/plain", "Dance sequence activated");
    });

server.on("/sensors", HTTP_GET, []() {
    Serial.println("Sensor endpoint hit!"); // Add this line
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


    server.begin();
    Serial.println("HTTP server started. Ready for commands.");
}


// ISR implementations
void IRAM_ATTR onLeftEncoder() {
  leftPulseCount++;
}

void IRAM_ATTR onRightEncoder() {
  rightPulseCount++;
}

void loop() {
    server.handleClient();  // Handle client requests to the web server

    static unsigned long lastReportTime = 0;
    const unsigned long reportInterval = 1000;  // Report every 1000 milliseconds (1 second)
    unsigned long currentTime = millis();

    // Check if it's time to report the sensor status
    if (currentTime - lastReportTime >= reportInterval) {
        lastReportTime = currentTime;  // Update the last report time

        // Autonomous or manual control checks
        if (!isManualControl) {
            autoMove();  // Perform autonomous movements
        }

        // Read IR sensor values
        IR_Left_Value = digitalRead(IR_LEFT);
        IR_Right_Value = digitalRead(IR_RIGHT);

        // Get the current distance from the ultrasonic sensor
        distance = getUltrasonicDistance();

        // Print IR sensor status and ultrasonic distance to the serial
        Serial.print("IR Left Value: ");
        Serial.print(IR_Left_Value);
        Serial.print(" | IR Right Value: ");
        Serial.print(IR_Right_Value);
        Serial.print(" | Distance: ");
        Serial.print(distance);
        Serial.println(" cm");
    }
}

void autoMove() {
    static unsigned long lastActionTime = 0;
    static int state = 0;
    unsigned long currentTime = millis();

    // First, update sensor readings without delay
    IR_Left_Value = digitalRead(IR_LEFT);
    IR_Right_Value = digitalRead(IR_RIGHT);
    distance = getUltrasonicDistance();

    // State machine handling different movement phases
    switch (state) {
        case 0:  // Check surroundings
            if (distance < 30) {
                handleFrontObstacle();
                state = 1;  // Go to thinking state after obstacle
            } else if (IR_Left_Value == 0 || IR_Right_Value == 0) {
                handleRearObstacle();
                state = 1;
            } else {
                state = 2;  // Safe to move, decide action
            }
            lastActionTime = currentTime;
            break;
        case 1:  // Thinking pause state
            if (currentTime - lastActionTime > 500) {
                state = 2;
            }
            break;
        case 2:  // Decide and act
            int action = random(0, 100);
            if (action < 20) {
                moveForward();
            } else if (action < 40) {
                moveBackward();
            } else if (action < 60) {
                spinRight();
            } else if (action < 80) {
                spinLeft();
            } else {
                danceRoutine();
            }
            state = 0;
            lastActionTime = currentTime;
            break;
    }
}

void exploreEnvironment() {
    static unsigned long lastActionTime = 0;
    static int state = 0;
    unsigned long currentTime = millis();

    // Update sensor readings once at the start of each cycle
    if (state == 0) {
        IR_Left_Value = digitalRead(IR_LEFT);
        IR_Right_Value = digitalRead(IR_RIGHT);
        distance = getUltrasonicDistance();
        state = 1;
    }

    // State machine for non-blocking exploration
    switch (state) {
        case 1: // Analyze environment
            if (distance > 100) {
                moveForward();
                lastActionTime = currentTime;
                state = 2;
            } else if (distance < 30) {
                handleFrontObstacle();
                state = 3;
            }
            break;
        case 2: // Moving forward
            if (currentTime - lastActionTime > 1000) {
                state = 0;
            }
            break;
        case 3: // Handling obstacle
            if (currentTime - lastActionTime > 500) {
                state = 0;
            }
            break;
    }
}

String controlPage() {
    String html = R"(
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
  body { font-family: Arial, sans-serif; background: #e0e5ec; display: flex; flex-direction: column; align-items: center; height: 100vh; margin: 0; }
  .button { border: none; border-radius: 12px; padding: 20px 40px; font-size: 16px; color: #fff; cursor: pointer; outline: none; margin: 10px; }
  .stop { background: #ff4136; }
  .control { background: #7fdbff; }
  .special { background: #85144b; }
  .button:active { color: #000; }
  #controlGrid { display: grid; grid-template-rows: auto auto auto; grid-template-columns: auto auto auto; justify-content: center; align-items: center; grid-gap: 10px; }
  #autoButton { grid-column: 1; grid-row: 1; }
  #exploreButton { grid-column: 3; grid-row: 1; }
  #forwardButton { grid-column: 2; grid-row: 1; }
  #leftButton { grid-column: 1; grid-row: 2; }
  #stopButton { grid-column: 2; grid-row: 2; }
  #rightButton { grid-column: 3; grid-row: 2; }
  #backwardButton { grid-column: 2; grid-row: 3; }
  #danceButton { grid-column: 1; grid-row: 3; }
  iframe { width: 80%; height: 200px; border-radius: 12px; border: none; margin-top: 20px; }
</style>
</head>
<body>
<h1>Robot Control Interface</h1>
<div id="controlGrid">
  <button id="autoButton" class="button special" onclick='sendCommand("/auto"); return false;'>Auto</button>
  <button id="exploreButton" class="button special" onclick='sendCommand("/explore"); return false;'>Explore</button>
  <button id="forwardButton" class="button control" onmousedown='sendCommand("/forward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Forward</button>
  <button id="leftButton" class="button control" onmousedown='sendCommand("/left"); return false;' onmouseup='sendCommand("/stop"); return false;'>Left</button>
  <button id="stopButton" class="button stop" onclick='sendCommand("/stop"); return false;'>Stop</button>
  <button id="rightButton" class="button control" onmousedown='sendCommand("/right"); return false;' onmouseup='sendCommand("/stop"); return false;'>Right</button>
  <button id="backwardButton" class="button control" onmousedown='sendCommand("/backward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Backward</button>
  <button id="danceButton" class="button special" onclick='sendCommand("/dance"); return false;'>Dance</button>
</div>
<iframe id="logFrame" srcdoc="<p>Command log initialized...</p>"></iframe>
<script>
function sendCommand(command) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', command, true);
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4 && xhr.status == 200) {
      document.getElementById('logFrame').contentWindow.document.body.innerHTML += '<p>' + command + ' command executed.</p>';
    }
  };
  xhr.send();
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
    delayMicroseconds(2); // Keep as it is needed for sensor timing
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10); // Keep as it is needed for sensor timing
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH, 38000); // Timeout added to avoid waiting too long
    distance = duration * 0.034 / 2;
    Serial.print("Distance measured: ");
    Serial.println(distance);
    return distance;
}

void handleFrontObstacle() {
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    static int state = 0;

    switch (state) {
        case 0: // Initial state, stop motors
            stopMotors();
            lastTime = currentTime;
            state = 1;
            break;
        case 1: // Delay for reaction time
            if (currentTime - lastTime > 1500) {
                state = 2;
            }
            break;
        case 2: // Check for head clearance and decide next step
            if (distance < 30) { // Assuming 30 cm is too close
                Serial.println("Obstacle might hit the head, extra precautions taken.");
                state = 3; // Additional safety maneuvers
            } else {
                Serial.println("Lower obstacle detected, normal avoidance maneuver.");
                state = 0; // Reset and allow normal operations
                if (IR_Left_Value == 0 || IR_Right_Value == 0) {
                    Serial.println("Obstacle detected at rear, spinning instead of reversing...");
                    state = 4; // Choose spin direction
                } else {
                    Serial.println("Rear path clear, reversing...");
                    moveBackward();
                    state = 5; // Delay reversing
                }
            }
            break;
        case 4: // Spinning to avoid reversing onto obstacle
            if (IR_Left_Value == 0) {
                spinRight();
            } else {
                spinLeft();
            }
            state = 0; // Reset after spin
            break;
        case 5: // Delay for reversing
            if (currentTime - lastTime > 2000) {
                stopMotors();
                state = 0; // Reset after reversing
            }
            break;
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

    // Dance move 1: Spin in place
    Serial.println("Dance move 1: Spin right");
    spinRight();
    delay(1000);
    Serial.println("Dance move 1: Spin left");
    spinLeft();
    delay(1000);
    stopMotors();

    // Dance move 2: Jiggle back and forth
    Serial.println("Dance move 2: Jiggle back and forth");
    for (int i = 0; i < 5; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }
    stopMotors();

    // Dance move 3: Circle spin
    Serial.println("Dance move 3: Circle spin");
    spinRight();
    delay(3000); // Longer spin
    stopMotors();

    // Dance move 4: Back and forth
    Serial.println("Dance move 4: Move forward and backward");
    moveForward();
    delay(1000);
    moveBackward();
    delay(1000);
    stopMotors();

    Serial.println("Dance routine complete!");
    server.send(200, "text/plain", "Dance complete");
    isManualControl = false; // Ensure control state is reset after dancing
}
