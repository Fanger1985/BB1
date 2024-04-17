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
#define IR_LEFT 13
#define IR_RIGHT 4

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
    server.on("/dance", HTTP_GET, []() {
    isManualControl = true; // Take manual control for the dance
    danceRoutine();
    server.send(200, "text/plain", "Dance sequence activated");
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

    if (!isManualControl) {
        // Autonomous behavior
        autoMove();
    }
    // No else needed; manual commands are handled via the web server routes
}

void autoMove() {
    // Autonomous movement logic based on sensor readings
    distance = getUltrasonicDistance();

    if (distance < 30) {  // If an obstacle is too close, take action
        handleFrontObstacle();
    } else {
        // If no close obstacles, randomly decide what to do next
        int randomChoice = random(0, 4);
        switch (randomChoice) {
            case 0:
                moveForward();
                break;
            case 1:
                spinLeft();
                break;
            case 2:
                spinRight();
                break;
            case 3:
                stopMotors();
                break;
        }
        delay(random(1000, 5000));  // Random delay for more dynamic behavior
    }
}

String controlPage() {
    String html = R"(
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
  body { font-family: Arial, sans-serif; background: #e0e5ec; display: flex; flex-direction: column; align-items: center; height: 100vh; margin: 0; }
  .button { border: none; background: #e0e5ec; border-radius: 12px; padding: 20px 40px; font-size: 16px; color: #333; box-shadow: -5px -5px 10px #fff, 5px 5px 15px #babecc; cursor: pointer; outline: none; margin: 10px; }
  .button:active { box-shadow: inset -5px -5px 10px #fff, inset 5px 5px 15px #babecc; color: #000; }
  #controlGrid { display: grid; grid-template-columns: auto auto auto; justify-content: center; align-items: center; grid-gap: 10px; }
  #stopButton { grid-column: 2; grid-row: 2; }
  #forwardButton { grid-column: 2; grid-row: 1; }
  #backwardButton { grid-column: 2; grid-row: 3; }
  #leftButton { grid-column: 1; grid-row: 2; }
  #rightButton { grid-column: 3; grid-row: 2; }
  #danceButton { grid-column: 3; grid-row: 3; }
  iframe { width: 80%; height: 200px; border-radius: 12px; border: none; box-shadow: -5px -5px 10px #fff, 5px 5px 15px #babecc; margin-top: 20px; }
</style>
</head>
<body>
<h1>Robot Control Interface</h1>
<div id="controlGrid">
  <button id="forwardButton" class="button" onmousedown='sendCommand("/forward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Forward</button>
  <button id="leftButton" class="button" onmousedown='sendCommand("/left"); return false;' onmouseup='sendCommand("/stop"); return false;'>Left</button>
  <button id="stopButton" class="button" onclick='sendCommand("/stop"); return false;'>Stop</button>
  <button id="rightButton" class="button" onmousedown='sendCommand("/right"); return false;' onmouseup='sendCommand("/stop"); return false;'>Right</button>
  <button id="backwardButton" class="button" onmousedown='sendCommand("/backward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Backward</button>
  <button id="danceButton" class="button" onclick='sendCommand("/dance"); return false;'>Dance</button>
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
    Serial.println("Moving backward...");
    digitalWrite(IN1_LEFT, LOW);
    digitalWrite(IN2_LEFT, HIGH);
    digitalWrite(IN1_RIGHT, LOW);
    digitalWrite(IN2_RIGHT, HIGH);
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
    delay(500);
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
        delay(1000);
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
    delay(500);
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
    Serial.println("Checking space for dance...");
    if (!checkSpaceForDance()) {
        Serial.println("Not enough space to dance :(");
        server.send(200, "text/plain", "Not enough space to dance!");
        isManualControl = false; // Reset control state even if dance isn't executed
        return;
    }

    Serial.println("Let's dance!");
    // Dance move 1: Spin in place
    spinRight();
    delay(1000);
    spinLeft();
    delay(1000);
    stopMotors();

    // Dance move 2: Jiggle back and forth
    for (int i = 0; i < 5; i++) {
        moveForward();
        delay(200);
        moveBackward();
        delay(200);
    }
    stopMotors();

    // Dance move 3: Circle spin
    spinRight();
    delay(3000); // Longer spin
    stopMotors();

    // Dance move 4: Back and forth
    moveForward();
    delay(1000);
    moveBackward();
    delay(1000);
    stopMotors();

    Serial.println("Dance routine complete!");
    server.send(200, "text/plain", "Dance complete");
    isManualControl = false; // Ensure control state is reset after dancing
}

