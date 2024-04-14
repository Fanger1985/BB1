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
    String html = R"(
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
  body { font-family: Arial, sans-serif; background: #e0e5ec; display: flex; justify-content: center; align-items: center; height: 100vh; flex-direction: column; margin: 0; }
  .control-panel { display: grid; grid-template-columns: 1fr 1fr 1fr; grid-template-rows: 1fr 1fr 1fr; gap: 10px; }
  .button { border: none; background: #e0e5ec; border-radius: 12px; padding: 20px; font-size: 16px; color: #333; box-shadow: -5px -5px 10px #fff, 5px 5px 15px #babecc; cursor: pointer; outline: none; }
  .button:active { box-shadow: inset -5px -5px 10px #fff, inset 5px 5px 15px #babecc; color: #000; }
  iframe { width: 80%; height: 200px; border-radius: 12px; border: none; box-shadow: -5px -5px 10px #fff, 5px 5px 15px #babecc; margin-top: 20px; }
</style>
</head>
<body>
<h1>Robot Control Interface</h1>
<div class="control-panel">
  <div></div>
  <button class="button" onmousedown='sendCommand("/forward")' onmouseup='sendCommand("/stop")'>Forward</button>
  <div></div>
  <button class="button" onmousedown='sendCommand("/left")' onmouseup='sendCommand("/stop")'>Left</button>
  <button class="button" onclick='sendCommand("/stop")'>Stop</button>
  <button class="button" onmousedown='sendCommand("/right")' onmouseup='sendCommand("/stop")'>Right</button>
  <div></div>
  <button class="button" onmousedown='sendCommand("/backward")' onmouseup='sendCommand("/stop")'>Backward</button>
  <div></div>
</div>
<iframe id="logFrame" srcdoc="<p>Command log initialized...</p>"></iframe>
<script>
function sendCommand(command) {
  var xhr = new XMLHttpRequest();
  xhr.open('GET', command, true);
  xhr.onreadystatechange = function() {
    if (xhr.readyState == 4 && xhr.status == 200) {
      // Use insertAdjacentHTML with 'afterbegin' to add new log entries at the top
      document.getElementById('logFrame').contentWindow.document.body.insertAdjacentHTML('afterbegin', '<p>' + command + ' command executed.</p>');
    }
  };
  xhr.send();
  return false; // Prevent default action, stop navigation
}
</script>
</body>
</html>
)";
    server.send(200, "text/html", html);
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
    server.handleClient();  // Handle client requests to the server

    if (!isManualControl) {
        // Autonomous behavior
        autoMove();
    }
    // No else needed; manual commands are handled via the web server routes
}

void autoMove() {
    // Autonomous movement logic as previously described
    IR_Left_Value = digitalRead(IR_LEFT);
    IR_Right_Value = digitalRead(IR_RIGHT);
    distance = getUltrasonicDistance();

    if (distance < 30) {
        handleFrontObstacle();
    } else if (IR_Left_Value == 0 || IR_Right_Value == 0) {
        handleRearObstacle();
    } else {
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
        delay(random(1000, 5000));  // Add random delay for more dynamic behavior
    }
}
String controlPage() {
    String html = R"(
<html>
<head>
<title>ESP32 Robot Control</title>
<style>
  body { font-family: Arial, sans-serif; background: #e0e5ec; display: flex; flex-direction: column; align-items: center; height: 100vh; margin: 0; }
  button { border: none; background: #e0e5ec; border-radius: 12px; padding: 20px 40px; font-size: 16px; color: #333; box-shadow: -5px -5px 10px #fff, 5px 5px 15px #babecc; cursor: pointer; outline: none; margin: 10px; }
  button:active { box-shadow: inset -5px -5px 10px #fff, inset 5px 5px 15px #babecc; color: #000; }
  iframe { width: 80%; height: 200px; border-radius: 12px; border: none; box-shadow: -5px -5px 10px #fff, 5px 5px 15px #babecc; margin-top: 20px; }
</style>
</head>
<body>
<h1>Robot Control Interface</h1>
<div>
  <button onmousedown='sendCommand("/forward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Forward</button>
  <button onmousedown='sendCommand("/backward"); return false;' onmouseup='sendCommand("/stop"); return false;'>Backward</button>
  <button onmousedown='sendCommand("/left"); return false;' onmouseup='sendCommand("/stop"); return false;'>Left</button>
  <button onmousedown='sendCommand("/right"); return false;' onmouseup='sendCommand("/stop"); return false;'>Right</button>
  <button onclick='sendCommand("/stop"); return false;'>Stop</button>
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
