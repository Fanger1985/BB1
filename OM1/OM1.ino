// Include necessary libraries
#include <Wire.h>
#include <I2Cdev.h>

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#include <MPU6050_6Axis_MotionApps20.h>

// Other includes
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>
#include <NewPing.h>  // Include NewPing library
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// New BLE libraries for Follow Me functionality
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>


// Wi-Fi credentials (replace with your actual credentials)
const char* ssid = "INADAZE";       // Replace with your Wi-Fi SSID
const char* password = "jeansrocket543";     // Replace with your Wi-Fi Password

// Reconnection settings
const unsigned long WS_RECONNECT_INTERVAL = 10000; // 10 seconds
unsigned long lastReconnectAttempt = 0;
bool wsClientConnected = false;

// Motor control pins (DRV8871)
#define MOTOR_LEFT_IN1 16   // MCPWM0A
#define MOTOR_LEFT_IN2 17   // MCPWM0B
#define MOTOR_RIGHT_IN1 25  // MCPWM1A
#define MOTOR_RIGHT_IN2 26  // MCPWM1B

#define MPU6050_INCLUDE_DMP_MOTIONAPPS20
#define PACKET_SIZE 42 // DMP packet size

// Debounce time for Hall sensors
const unsigned long debounceTimeMicros = 10; // Adjust the value as needed

bool smartNavEnabled = false;  // SMART NAV flag

// Timing intervals
const unsigned long WS_SEND_INTERVAL = 3000;  // WebSocket send interval, adjust as needed
const unsigned long HEAP_PRINT_INTERVAL = 6000; // Print free heap every 6 seconds

// PWM Configuration
#define PWM_FREQUENCY 20000       // 20kHz frequency for PWM
#define PWM_MAX_DUTY_CYCLE 100.0f // Max duty cycle percentage

// Speed levels
const float LOW_SPEED = 70.0f;
const float HIGH_SPEED = 90.0f;
const float NORMAL_SPEED = 90.0f;
float currentSpeed = NORMAL_SPEED; // Variable to keep track of current speed
// Global variables for gyro-based control
float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

// Ultrasonic sensor setup using NewPing library
#define MAX_DISTANCE 200 // Maximum distance (in cm)

// Define trigger and echo pins for each sensor
#define FRONT_TRIGGER 4
#define FRONT_ECHO 13

#define LEFT_TRIGGER 19
#define LEFT_ECHO 33

#define RIGHT_TRIGGER 18
#define RIGHT_ECHO 12  // Changed from GPIO12 to GPIO21 to avoid boot issues

#define BACK_TRIGGER 5
#define BACK_ECHO 32

// Create NewPing objects for each sensor
NewPing frontSensor(FRONT_TRIGGER, FRONT_ECHO, MAX_DISTANCE);
NewPing leftSensor(LEFT_TRIGGER, LEFT_ECHO, MAX_DISTANCE);
NewPing rightSensor(RIGHT_TRIGGER, RIGHT_ECHO, MAX_DISTANCE);
NewPing backSensor(BACK_TRIGGER, BACK_ECHO, MAX_DISTANCE);

// Define states for faceOpenArea
enum FaceOpenAreaState {
    FACE_IDLE,
    FACE_TURNING_BACKWARD,
    FACE_TURNING_LEFT,
    FACE_TURNING_RIGHT,
    FACE_DONE
};
enum TurnState {
    TURN_IDLE,
    TURN_IN_PROGRESS
};

// Ultrasonic sensor distances
int frontDistance = MAX_DISTANCE;
int rearDistance = MAX_DISTANCE;
int leftDistance = MAX_DISTANCE;
int rightDistance = MAX_DISTANCE;

// Define safe distances
#define SAFE_DISTANCE 55  // Safe distance in cm

// MPU6050 setup and variables
MPU6050 mpu;

// DMP Variables
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // Count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

// Web server and WebSocket server
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Movement function prototypes
void moveForward(float speedPercentage = NORMAL_SPEED);
void moveBackward(float speedPercentage = NORMAL_SPEED);
void spinLeft(float speedPercentage = NORMAL_SPEED);
void spinRight(float speedPercentage = NORMAL_SPEED);
void stopMotors();
void autoMove();
bool reactToCloseObstacleNonBlocking();
void taskA();
void performTaskB();
void performTaskC();
void sendSensorData();
void handleExpressEmotion(AsyncWebServerRequest *request);
void checkIfStuck();
void initMCPWM();
void setMotorSpeed(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle_A, float duty_cycle_B);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void danceRoutine();
void faceOpenArea();
void updateGyroData();
void checkWebSocketHealth();
void readUltrasonicSensors();
void followBeacon();  // New function for Follow Me mode
void navigateToBeacon(int rssi); // New function for navigation based on RSSI
void preciseTurn(float targetYaw, float speedPercentage = NORMAL_SPEED);

// ISR functions for Hall sensors

// Movement flags
volatile bool movingForwardFlag = false; // Declare as volatile for interrupt safety

// Hall sensor pulse counts
volatile int leftHallPulseCount = 0;
volatile int rightHallPulseCount = 0;
int lastLeftPulseCount = 0;
int lastRightPulseCount = 0;
bool isStuck = false;

// ISR function prototypes for Hall sensors
void IRAM_ATTR onLeftHallSensor1();
void IRAM_ATTR onLeftHallSensor2();
void IRAM_ATTR onRightHallSensor1();
void IRAM_ATTR onRightHallSensor2();

void IRAM_ATTR onLeftHallSensor1() {
    leftHallPulseCount++;
}

void IRAM_ATTR onLeftHallSensor2() {
    leftHallPulseCount++;
}

void IRAM_ATTR onRightHallSensor1() {
    rightHallPulseCount++;
}

void IRAM_ATTR onRightHallSensor2() {
    rightHallPulseCount++;
}

// Define states for autoMove
enum AutoMoveState {
    AUTO_MOVE_IDLE,
    AUTO_MOVE_FORWARD,
    AUTO_MOVE_ADJUST_LEFT,
    AUTO_MOVE_ADJUST_RIGHT,
    AUTO_MOVE_REACT_OBSTACLE
};

AutoMoveState autoMoveState = AUTO_MOVE_IDLE;
unsigned long autoMoveStateStartTime = 0;

// Define states for reactToCloseObstacle
enum ReactObstacleState {
    REACT_START,
    REACT_MOVE_BACKWARD,
    REACT_SPIN,
    REACT_COMPLETE
};

ReactObstacleState reactState = REACT_COMPLETE;
unsigned long reactStateStartTime = 0;

// Define states for dance routine
enum DanceState {
    DANCE_IDLE,
    DANCE_MOVE_FORWARD,
    DANCE_SPIN_LEFT,
    DANCE_MOVE_FORWARD2,
    DANCE_SPIN_RIGHT,
    DANCE_JIGGLE,
    DANCE_MOONWALK,
    DANCE_SPIN_WIGGLE,
    DANCE_COMPLETE
};

DanceState danceState = DANCE_IDLE;
unsigned long danceStateStartTime = 0;
int jiggleCount = 0;
int wiggleCount = 0;
bool danceInitiated = false;

// Define states for Task A's chill mode
enum TaskAState {
    TASK_A_IDLE,
    TASK_A_MOVE_FORWARD,
    TASK_A_ADJUST_LEFT,
    TASK_A_ADJUST_RIGHT,
    TASK_A_CHILL,
    TASK_A_REACT_OBSTACLE
};

TaskAState taskAState = TASK_A_IDLE;
unsigned long taskAStateStartTime = 0;

// Define states for Task B patrol
enum TaskBState {
    TASK_B_IDLE,
    TASK_B_MOVE_FORWARD,
    TASK_B_TURN_AROUND
};

TaskBState taskBState = TASK_B_IDLE;
unsigned long taskBStateStartTime = 0;

// Define mode management
enum RobotMode {
    MODE_MANUAL,
    MODE_AUTO_MOVE,
    MODE_TASK_A,
    MODE_TASK_B,
    MODE_TASK_C,
    MODE_DANCE,
    MODE_FOLLOW_ME  // New mode for Follow Me
};

RobotMode currentMode = MODE_MANUAL;

// Global variables for duty cycles
float leftMotorDutyCycle = 0.0f;
float rightMotorDutyCycle = 0.0f;

// Timing variables
unsigned long lastMPUReadTime = 0;
const unsigned long MPU_READ_INTERVAL = 10; // Read every 50 ms

unsigned long lastSerialPrintTime = 0;
const unsigned long SERIAL_PRINT_INTERVAL = 500; // Print every 500 ms

void taskServer(void *parameter);
void taskSensorControl(void *parameter);

TaskHandle_t serverTaskHandle = NULL;
TaskHandle_t sensorControlTaskHandle = NULL;

// New BLE variables for Follow Me functionality
#define SCAN_TIME 2 // BLE scan duration in seconds
BLEScan* pBLEScan;

String followBeaconUUID = "12345678-1234-1234-1234-123456789abc"; // Beacon UUID
int followRSSIThreshold = -50; // RSSI threshold to consider "close enough"

void setup() {
    Serial.begin(115200);

    // Initialize mode states
    reactState = REACT_COMPLETE;
    autoMoveState = AUTO_MOVE_IDLE;
    taskAState = TASK_A_IDLE;
    taskBState = TASK_B_IDLE;

    // Wi-Fi connection setup
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi...");
    int wifi_attempts = 0;
    while (WiFi.status() != WL_CONNECTED && wifi_attempts < 20) {
        delay(500);
        Serial.print(".");
        wifi_attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi, IP: " + WiFi.localIP().toString());
        if (!MDNS.begin("BB2mobileunit")) {
            Serial.println("Error setting up mDNS responder!");
        }
    } else {
        Serial.println("Failed to connect to WiFi after multiple attempts.");
    }
   // MPU6050 initialization
    Serial.println("Initializing MPU6050...");
    Wire.begin();
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed.");
    } else {
        Serial.println("MPU6050 connected.");
    }

    // Initialize MPU6050 with DMP
    if (mpu.dmpInitialize() == 0) {
        // Turn on the DMP, now that it's ready
mpu.setDMPEnabled(true);
mpu.setRate(19); // Adjust output rate; lower value = higher frequency (default is 0)
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP ready.");
    } else {
        Serial.println("DMP Initialization failed.");
    }


    // Ultrasonic sensors setup
    Serial.println("Setting up ultrasonic sensors...");
    // Pin modes are handled by the NewPing library, but you can set them explicitly if desired
    pinMode(FRONT_TRIGGER, OUTPUT);
    pinMode(FRONT_ECHO, INPUT);

    pinMode(LEFT_TRIGGER, OUTPUT);
    pinMode(LEFT_ECHO, INPUT);

    pinMode(RIGHT_TRIGGER, OUTPUT);
    pinMode(RIGHT_ECHO, INPUT);

    pinMode(BACK_TRIGGER, OUTPUT);
    pinMode(BACK_ECHO, INPUT);

    // Motor control setup with MCPWM
    Serial.println("Initializing motor control (MCPWM)...");
    initMCPWM();

    // Hall sensors setup with interrupts
    Serial.println("Setting up Hall sensors...");
    pinMode(34, INPUT_PULLUP);
    pinMode(35, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    pinMode(27, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(34), onLeftHallSensor1, RISING);
    attachInterrupt(digitalPinToInterrupt(35), onLeftHallSensor2, RISING);
    attachInterrupt(digitalPinToInterrupt(14), onRightHallSensor1, RISING);
    attachInterrupt(digitalPinToInterrupt(27), onRightHallSensor2, RISING);

    // WebSocket and HTTP server setup
    ws.onEvent(onEvent);
    server.addHandler(&ws);

    // HTTP routes setup for basic commands
    server.on("/forward", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_MANUAL;
        moveForward(currentSpeed);
        request->send(200, "text/plain", "Moving forward");
    });
    server.on("/backward", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_MANUAL;
        moveBackward(currentSpeed);
        request->send(200, "text/plain", "Moving backward");
    });
    server.on("/left", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_MANUAL;
        spinLeft(currentSpeed);
        request->send(200, "text/plain", "Turning left");
    });
    server.on("/right", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_MANUAL;
        spinRight(currentSpeed);
        request->send(200, "text/plain", "Turning right");
    });
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_MANUAL;
        stopMotors();
        request->send(200, "text/plain", "Stopping");
    });
    server.on("/speed/low", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentSpeed = LOW_SPEED;
        request->send(200, "text/plain", "Speed set to LOW");
    });
    server.on("/speed/normal", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentSpeed = NORMAL_SPEED;
        request->send(200, "text/plain", "Speed set to NORMAL");
    });
    server.on("/speed/high", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentSpeed = HIGH_SPEED;
        request->send(200, "text/plain", "Speed set to HIGH");
    });
    server.on("/smartnav/on", HTTP_GET, [](AsyncWebServerRequest *request) {
        smartNavEnabled = true;
        request->send(200, "text/plain", "SMART NAV activated");
    });
    server.on("/smartnav/off", HTTP_GET, [](AsyncWebServerRequest *request) {
        smartNavEnabled = false;
        stopMotors();
        request->send(200, "text/plain", "SMART NAV deactivated");
    });
    server.on("/auto", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_AUTO_MOVE;
        autoMoveState = AUTO_MOVE_IDLE; // Reset state
        request->send(200, "text/plain", "Switched to auto mode");
    });
    server.on("/dance", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_DANCE;
        danceState = DANCE_IDLE; // Start dance routine
        danceInitiated = true;
        request->send(200, "text/plain", "Dance sequence activated");
    });
    server.on("/task_a", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_TASK_A;
        taskAState = TASK_A_IDLE;
        request->send(200, "text/plain", "Task A activated - Chill Mode with orientation");
    });
    server.on("/task_b", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_TASK_B;
        taskBState = TASK_B_IDLE;
        request->send(200, "text/plain", "Task B activated");
    });
    server.on("/task_c", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_TASK_C;
        performTaskC();
        request->send(200, "text/plain", "Task C activated");
    });
    server.on("/expressEmotion", HTTP_POST, handleExpressEmotion); // Placeholder

    // New endpoint to activate Follow Me mode
    server.on("/follow_me", HTTP_GET, [](AsyncWebServerRequest *request) {
        currentMode = MODE_FOLLOW_ME;
        request->send(200, "text/plain", "Follow Me mode activated.");
    });

    // Additional endpoints for any custom commands
    server.on("/sensorStatus", HTTP_GET, [](AsyncWebServerRequest *request) {
        StaticJsonDocument<200> jsonDoc;
        jsonDoc["frontDistance"] = frontDistance;
        jsonDoc["leftDistance"] = leftDistance;
        jsonDoc["rightDistance"] = rightDistance;
        jsonDoc["rearDistance"] = rearDistance;
        String jsonStr;
        serializeJson(jsonDoc, jsonStr);
        request->send(200, "application/json", jsonStr);
    });

    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        ESP.restart();
        request->send(200, "text/plain", "Rebooting...");
    });

    // Start HTTP server
    server.begin();
    Serial.println("HTTP server started. Ready for commands.");

    // Initialize BLE for Follow Me functionality
    BLEDevice::init("BB1-Follower");
    pBLEScan = BLEDevice::getScan(); // Create a BLE scanner
    pBLEScan->setActiveScan(true);   // Enable active scanning for better results
    Serial.println("BLE initialized for Follow Me mode.");

    // Print free heap memory after setup
    Serial.print("Free heap after setup: ");
    Serial.println(ESP.getFreeHeap());

    // Task creation for Core 0 and Core 1 with increased stack sizes
    xTaskCreatePinnedToCore(taskServer, "TaskServer", 8192, NULL, 2, &serverTaskHandle, 0); // Increased priority
    xTaskCreatePinnedToCore(taskSensorControl, "TaskSensorControl", 8192, NULL, 1, &sensorControlTaskHandle, 1); // Sensor control and main loop on Core 1

    Serial.println("Setup complete");
}

void taskServer(void *parameter) {
    Serial.println("Server task started on Core 0.");

    unsigned long lastHeapPrintTime = 0;

    for (;;) {
        ws.cleanupClients();

        // Print free heap memory periodically
        if (millis() - lastHeapPrintTime >= HEAP_PRINT_INTERVAL) {
            lastHeapPrintTime = millis();
            Serial.print("[TaskServer] Free heap memory: ");
            Serial.println(ESP.getFreeHeap());
        }

        // Use FreeRTOS delay
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void taskSensorControl(void *parameter) {
    unsigned long lastWsSendTime = 0;
    unsigned long lastStuckCheckTime = 0;
    const unsigned long STUCK_CHECK_INTERVAL = 1000; // Check every 1 second
    unsigned long lastHeapPrintTime = 0;
    unsigned long lastSensorReadTime = 0;
    const unsigned long SENSOR_READ_INTERVAL = 50; // Adjust as needed
    uint8_t currentSensor = 0;

    for (;;) {
        unsigned long currentMillis = millis();

        // Update gyro data
        updateGyroData();

        // Ultrasonic sensor readings
        if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
            lastSensorReadTime = currentMillis;

            // Trigger one sensor at a time
            switch (currentSensor) {
                case 0:
                    frontDistance = frontSensor.ping_cm();
                    Serial.print("Front Distance: "); Serial.println(frontDistance);
                    break;
                case 1:
                    leftDistance = leftSensor.ping_cm();
                    Serial.print("Left Distance: "); Serial.println(leftDistance);
                    break;
                case 2:
                    rightDistance = rightSensor.ping_cm();
                    Serial.print("Right Distance: "); Serial.println(rightDistance);
                    break;
                case 3:
                    rearDistance = backSensor.ping_cm();
                    Serial.print("Rear Distance: "); Serial.println(rearDistance);
                    break;
            }

            currentSensor = (currentSensor + 1) % 4; // Move to next sensor
        }

        // Send sensor data over WebSocket
        if (ws.count() > 0 && currentMillis - lastWsSendTime >= WS_SEND_INTERVAL) {
            lastWsSendTime = currentMillis;
            sendSensorData();
        }

        // Execute function for the current mode only
        switch (currentMode) {
            case MODE_MANUAL:
                // Do nothing in manual mode unless commands are received
                break;
            case MODE_AUTO_MOVE:
                autoMove();
                if (currentMillis - lastStuckCheckTime >= STUCK_CHECK_INTERVAL) {
                    lastStuckCheckTime = currentMillis;
                    checkIfStuck();
                }
                break;
            case MODE_TASK_A:
                taskA();
                break;
            case MODE_TASK_B:
                performTaskB();
                break;
            case MODE_TASK_C:
                performTaskC();
                break;
            case MODE_DANCE:
                danceRoutine();
                break;
            case MODE_FOLLOW_ME:
                followBeacon();
                break;
            default:
                break;
        }

        // Print free heap memory periodically
        if (currentMillis - lastHeapPrintTime >= HEAP_PRINT_INTERVAL) {
            lastHeapPrintTime = currentMillis;
            Serial.print("[TaskSensorControl] Free heap memory: ");
            Serial.println(ESP.getFreeHeap());
        }

        // Use FreeRTOS delay
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void updateGyroData() {
    // Get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // FIFO overflow! Reset the FIFO
        mpu.resetFIFO();
        Serial.println("FIFO overflow! Resetting FIFO buffer.");
    } else if (fifoCount >= packetSize) {
        // Read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Process the packet
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert yaw, pitch, roll from radians to degrees
        yaw   = ypr[0] * 180.0f / PI;
        pitch = ypr[1] * 180.0f / PI;
        roll  = ypr[2] * 180.0f / PI;

        // Keep yaw within -180 to 180 degrees
        if (yaw > 180) yaw -= 360;
        else if (yaw < -180) yaw += 360;
    }
}

// WebSocket health-check function to maintain stability
void checkWebSocketHealth() {
    unsigned long currentMillis = millis();

    if (!wsClientConnected && (currentMillis - lastReconnectAttempt >= WS_RECONNECT_INTERVAL)) {
        lastReconnectAttempt = currentMillis;
        ws.cleanupClients();
        Serial.println("Attempting to re-establish WebSocket connections...");
    }
}

void loop() {
    checkWebSocketHealth();
    vTaskDelay(pdMS_TO_TICKS(10));
}

void sendSensorData() {
    static int lastFront = MAX_DISTANCE, lastRear = MAX_DISTANCE;
    static int lastLeft = MAX_DISTANCE, lastRight = MAX_DISTANCE;

    if (ws.count() == 0) return;

    // Only send if there's a meaningful change
    if (abs(frontDistance - lastFront) > 5 ||
        abs(rearDistance - lastRear) > 5 ||
        abs(leftDistance - lastLeft) > 5 ||
        abs(rightDistance - lastRight) > 5) {

        lastFront = frontDistance;
        lastRear = rearDistance;
        lastLeft = leftDistance;
        lastRight = rightDistance;

        StaticJsonDocument<128> doc;
        doc["sensorData"]["front"] = frontDistance;
        doc["sensorData"]["rear"] = rearDistance;
        doc["sensorData"]["left"] = leftDistance;
        doc["sensorData"]["right"] = rightDistance;
    // Add yaw data
    doc["sensorData"]["yaw"] = yaw;

        String jsonString;
        serializeJson(doc, jsonString);

        ws.textAll(jsonString);
    }
}

// Initialize MCPWM
void initMCPWM() {
    // Initialize MCPWM units
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MOTOR_LEFT_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MOTOR_LEFT_IN2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, MOTOR_RIGHT_IN1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, MOTOR_RIGHT_IN2);

    // Configure MCPWM units
    mcpwm_config_t pwm_config;
    pwm_config.frequency = PWM_FREQUENCY;    // Frequency in Hz
    pwm_config.cmpr_a = 0;                   // Duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;                   // Duty cycle of PWMxB = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Initialize MCPWM units with the configuration
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

void setMotorSpeed(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num, float duty_cycle_A, float duty_cycle_B) {
    float leftMotorBias = 1.0;  // Bias for left motor
    float rightMotorBias = 0.98; // Bias for right motor

    // Apply biases based on motor side
    if (mcpwm_num == MCPWM_UNIT_0) { // Left motor
        duty_cycle_A *= leftMotorBias;
        duty_cycle_B *= leftMotorBias;
    } else if (mcpwm_num == MCPWM_UNIT_1) { // Right motor
        duty_cycle_A *= rightMotorBias;
        duty_cycle_B *= rightMotorBias;
    }

    // Set duty cycles for MCPWM_OPR_A
    if (duty_cycle_A >= 0) {
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle_A);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    }

    // Set duty cycles for MCPWM_OPR_B
    if (duty_cycle_B >= 0) {
        mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_B, duty_cycle_B);
        mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
    }
}

float targetYaw = 0.0; // Declare globally
void startMovingForward(float speedPercentage) {
    updateGyroData();
    targetYaw = yaw; // Set targetYaw to current yaw
    moveForward(speedPercentage);
}
// Movement functions
void moveForward(float speedPercentage) {
    float dutyCycle = PWM_MAX_DUTY_CYCLE * speedPercentage / 100.0f;
    leftMotorDutyCycle = dutyCycle;
    rightMotorDutyCycle = dutyCycle;

    // Set motor speeds
    setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, leftMotorDutyCycle, 0); // Left motor forward
    setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, 0, rightMotorDutyCycle); // Right motor forward
}


void moveBackward(float speedPercentage) {
    movingForwardFlag = false;

    float dutyCycle = PWM_MAX_DUTY_CYCLE * speedPercentage / 100.0f;
    leftMotorDutyCycle = dutyCycle;
    rightMotorDutyCycle = dutyCycle;

    // Left motor backward
    setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, leftMotorDutyCycle);
    // Right motor backward
    setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, rightMotorDutyCycle, 0);
}

void spinLeft(float speedPercentage) {
    movingForwardFlag = false;

    float dutyCycle = PWM_MAX_DUTY_CYCLE * speedPercentage / 100.0f;
    leftMotorDutyCycle = dutyCycle;
    rightMotorDutyCycle = dutyCycle;

    // Left motor backward
    setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, leftMotorDutyCycle);
    // Right motor forward
    setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, 0, rightMotorDutyCycle);
}

void spinRight(float speedPercentage) {
    movingForwardFlag = false;

    float dutyCycle = PWM_MAX_DUTY_CYCLE * speedPercentage / 100.0f;
    leftMotorDutyCycle = dutyCycle;
    rightMotorDutyCycle = dutyCycle;

    // Left motor forward
    setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, leftMotorDutyCycle, 0);
    // Right motor backward
    setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, rightMotorDutyCycle, 0);
}

void stopMotors() {
    movingForwardFlag = false;
    leftMotorDutyCycle = 0;
    rightMotorDutyCycle = 0;

    setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, 0, 0);
    setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, 0, 0);
}

// WebSocket event handler
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("WebSocket client connected");
        wsClientConnected = true;
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket client disconnected");
        wsClientConnected = false;
    } else if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len);
    }
}

// Handle incoming WebSocket messages
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;

    // Ensure the message is complete, text-based, and not fragmented
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = '\0'; // Null-terminate data to treat it as a string
        StaticJsonDocument<256> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, (char*)data);

        if (error) {
            Serial.print("JSON parse error: ");
            Serial.println(error.c_str());
            return; // Exit if JSON parsing fails
        }

        // Check if JSON contains the "command" key
        if (jsonDoc.containsKey("command")) {
            const char* command = jsonDoc["command"];

            // Command: Movement commands
            if (strcmp(command, "moveForward") == 0) {
                Serial.println("Moving forward");
                moveForward(currentSpeed);
            }
            else if (strcmp(command, "moveBackward") == 0) {
                Serial.println("Moving backward");
                moveBackward(currentSpeed);
            }
            else if (strcmp(command, "spinLeft") == 0) {
                Serial.println("Spinning left");
                spinLeft(currentSpeed);
            }
            else if (strcmp(command, "spinRight") == 0) {
                Serial.println("Spinning right");
                spinRight(currentSpeed);
            }
            else if (strcmp(command, "stop") == 0) {
                Serial.println("Stopping motors");
                stopMotors();
            }

            // Command: Speed adjustment
            else if (strcmp(command, "setSpeed") == 0) {
                if (jsonDoc.containsKey("value")) {
                    currentSpeed = jsonDoc["value"];
                    Serial.print("Speed set to ");
                    Serial.println(currentSpeed);
                } else {
                    Serial.println("Error: Missing 'value' for setSpeed command");
                }
            }

            // Command: Set robot mode
            else if (strcmp(command, "setMode") == 0) {
                if (jsonDoc.containsKey("mode")) {
                    const char* mode = jsonDoc["mode"];
                    if (strcmp(mode, "manual") == 0) {
                        currentMode = MODE_MANUAL;
                        Serial.println("Mode set to MANUAL");
                    } else if (strcmp(mode, "auto") == 0) {
                        currentMode = MODE_AUTO_MOVE;
                        Serial.println("Mode set to AUTO_MOVE");
                    } else if (strcmp(mode, "taskA") == 0) {
                        currentMode = MODE_TASK_A;
                        Serial.println("Mode set to TASK_A");
                    } else if (strcmp(mode, "taskB") == 0) {
                        currentMode = MODE_TASK_B;
                        Serial.println("Mode set to TASK_B");
                    } else if (strcmp(mode, "dance") == 0) {
                        currentMode = MODE_DANCE;
                        Serial.println("Mode set to DANCE");
                    } else if (strcmp(mode, "followMe") == 0) {
                        currentMode = MODE_FOLLOW_ME;
                        Serial.println("Mode set to FOLLOW_ME");
                    } else {
                        Serial.println("Error: Invalid mode");
                    }
                } else {
                    Serial.println("Error: Missing 'mode' key for setMode command");
                }
            }

            // Unknown command
            else {
                Serial.print("Error: Unknown command '");
                Serial.print(command);
                Serial.println("'");
            }
        } else {
            Serial.println("Error: No 'command' key in JSON");
        }
    }
}

// React to close obstacle
bool reactToCloseObstacleNonBlocking() {
    unsigned long currentMillis = millis();

    switch (reactState) {
        case REACT_START:
            stopMotors();
            moveBackward(LOW_SPEED);
            reactStateStartTime = currentMillis;
            reactState = REACT_MOVE_BACKWARD;
            break;

        case REACT_MOVE_BACKWARD:
            if (currentMillis - reactStateStartTime >= 500) {
                stopMotors();
                if (random(0, 2)) {
                    spinLeft(NORMAL_SPEED);
                } else {
                    spinRight(NORMAL_SPEED);
                }
                reactStateStartTime = currentMillis;
                reactState = REACT_SPIN;
            }
            break;

        case REACT_SPIN:
            if (currentMillis - reactStateStartTime >= 500) {
                stopMotors();
                reactState = REACT_COMPLETE;
                return true; // Reaction complete
            }
            break;

        case REACT_COMPLETE:
            // Do nothing
            return true; // Reaction complete
    }

    return false; // Reaction in progress
}

// Definition of autoMove() function
void autoMove() {
    if (currentMode != MODE_AUTO_MOVE) {
        return;  // Exit if not in AUTO_MOVE mode
    }

    unsigned long currentMillis = millis();

    switch (autoMoveState) {
        case AUTO_MOVE_IDLE:
            if (frontDistance > SAFE_DISTANCE) {
                Serial.println("Open space ahead, moving forward confidently...");
                autoMoveState = AUTO_MOVE_FORWARD;
                moveForward(NORMAL_SPEED);
            } else {
                Serial.println("Obstacle detected ahead, reacting...");
                autoMoveState = AUTO_MOVE_REACT_OBSTACLE;
                reactState = REACT_START; // Initialize react state
            }
            break;

        case AUTO_MOVE_FORWARD:
            if (frontDistance < SAFE_DISTANCE) {
                stopMotors();
                autoMoveState = AUTO_MOVE_REACT_OBSTACLE;
                reactState = REACT_START; // Initialize react state
            } else if (leftDistance < SAFE_DISTANCE) {
                Serial.println("Obstacle on the left, adjusting course to the right...");
                autoMoveState = AUTO_MOVE_ADJUST_RIGHT;
                autoMoveStateStartTime = currentMillis;
                moveForward(NORMAL_SPEED);
                setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, leftMotorDutyCycle * 0.8f, 0); // Reduce left motor speed
            } else if (rightDistance < SAFE_DISTANCE) {
                Serial.println("Obstacle on the right, adjusting course to the left...");
                autoMoveState = AUTO_MOVE_ADJUST_LEFT;
                autoMoveStateStartTime = currentMillis;
                moveForward(NORMAL_SPEED);
                setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, 0, rightMotorDutyCycle * 0.8f); // Reduce right motor speed
            } else {
                // Continue moving forward at normal speed
                moveForward(NORMAL_SPEED);
            }
            break;

        case AUTO_MOVE_ADJUST_RIGHT:
            if (currentMillis - autoMoveStateStartTime >= 500) { // Adjust for 500 ms
                autoMoveState = AUTO_MOVE_FORWARD;
            }
            break;

        case AUTO_MOVE_ADJUST_LEFT:
            if (currentMillis - autoMoveStateStartTime >= 500) { // Adjust for 500 ms
                autoMoveState = AUTO_MOVE_FORWARD;
            }
            break;

        case AUTO_MOVE_REACT_OBSTACLE:
            if (reactToCloseObstacleNonBlocking()) {
                autoMoveState = AUTO_MOVE_IDLE;
            }
            break;
    }
}

// Check if the robot is stuck
void checkIfStuck() {
    static unsigned long lastCheckTime = 0;
    const long checkInterval = 1000;

    unsigned long currentMillis = millis();

    if (currentMillis - lastCheckTime >= checkInterval) {
        noInterrupts();
        int leftCount = leftHallPulseCount;
        int rightCount = rightHallPulseCount;
        interrupts();

        if (lastLeftPulseCount == leftCount && lastRightPulseCount == rightCount) {
            if (!isStuck) {
                Serial.println("Robot might be stuck, stopping motors...");
                stopMotors();
                isStuck = true;
            }
        } else {
            lastLeftPulseCount = leftCount;
            lastRightPulseCount = rightCount;
            isStuck = false;
        }
        lastCheckTime = currentMillis;
    }
}


// Modified Task A with orientation in chill mode
void taskA() {
    unsigned long currentMillis = millis();
    if (currentMode != MODE_TASK_A) {
        return;  // Exit if not in Task A mode
    }
    switch (taskAState) {
        case TASK_A_IDLE:
            if (frontDistance > SAFE_DISTANCE && leftDistance > SAFE_DISTANCE && rightDistance > SAFE_DISTANCE) {
                Serial.println("Open space detected, preparing to chill...");
                faceOpenArea();  // Face the most open area
                stopMotors();
                taskAStateStartTime = currentMillis;
                taskAState = TASK_A_CHILL;
            } else if (frontDistance > SAFE_DISTANCE) {
                taskAState = TASK_A_MOVE_FORWARD;
                moveForward(NORMAL_SPEED);
            } else {
                taskAState = TASK_A_REACT_OBSTACLE;
                reactState = REACT_START;  // Initialize react state
            }
            break;

        case TASK_A_MOVE_FORWARD:
            if (frontDistance < SAFE_DISTANCE) {
                stopMotors();
                taskAState = TASK_A_REACT_OBSTACLE;
            } else if (leftDistance < SAFE_DISTANCE) {
                Serial.println("Obstacle on the left, adjusting course to the right...");
                taskAState = TASK_A_ADJUST_RIGHT;
                taskAStateStartTime = currentMillis;
                moveForward(NORMAL_SPEED);
                setMotorSpeed(MCPWM_UNIT_0, MCPWM_TIMER_0, leftMotorDutyCycle * 0.8f, 0); // Reduce left motor speed
            } else if (rightDistance < SAFE_DISTANCE) {
                Serial.println("Obstacle on the right, adjusting course to the left...");
                taskAState = TASK_A_ADJUST_LEFT;
                taskAStateStartTime = currentMillis;
                moveForward(NORMAL_SPEED);
                setMotorSpeed(MCPWM_UNIT_1, MCPWM_TIMER_1, 0, rightMotorDutyCycle * 0.8f); // Reduce right motor speed
            } else {
                // Continue moving forward at normal speed
                moveForward(NORMAL_SPEED);
            }
            break;

        case TASK_A_ADJUST_LEFT:
            if (currentMillis - taskAStateStartTime >= 500) {  // Adjust left for 500 ms
                taskAState = TASK_A_MOVE_FORWARD;
            }
            break;

        case TASK_A_ADJUST_RIGHT:
            if (currentMillis - taskAStateStartTime >= 500) {  // Adjust right for 500 ms
                taskAState = TASK_A_MOVE_FORWARD;
            }
            break;

        case TASK_A_CHILL:
            if (currentMillis - taskAStateStartTime >= 6666) {  // Chill for 3 seconds
                Serial.println("Chill time over, exploring again...");
                taskAState = TASK_A_IDLE;
            } else {
                stopMotors();
            }
            break;

        case TASK_A_REACT_OBSTACLE:
            if (reactToCloseObstacleNonBlocking()) {
                taskAState = TASK_A_IDLE;
            }
            break;
    }
}
// Modified Task B: Back and Forth Patrol
void performTaskB() {
    unsigned long currentMillis = millis();
    if (currentMode != MODE_TASK_B) {
        return;  // Exit if not in Task B mode
    }

    switch (taskBState) {
        case TASK_B_IDLE:
            Serial.println("Starting patrol...");
            taskBStateStartTime = currentMillis;
            taskBState = TASK_B_MOVE_FORWARD;
            moveForward(NORMAL_SPEED);
            break;

        case TASK_B_MOVE_FORWARD:
            if (currentMillis - taskBStateStartTime >= 3000) { // Move forward for 3 seconds
                stopMotors();
                taskBStateStartTime = currentMillis;
                taskBState = TASK_B_TURN_AROUND;
                spinRight(NORMAL_SPEED); // Start turning
            } else if (frontDistance < SAFE_DISTANCE) {
                stopMotors();
                reactState = REACT_START; // Initialize react state
                taskBState = TASK_B_TURN_AROUND; // Proceed to turn around
                spinRight(NORMAL_SPEED); // Start turning
            }
            break;

        case TASK_B_TURN_AROUND:
            if (currentMillis - taskBStateStartTime >= 5000) { // Turn for 1 second (adjust as needed) 1 second is a 1 hr turn maybe 2
                stopMotors();
                taskBStateStartTime = currentMillis;
                taskBState = TASK_B_MOVE_FORWARD; // Start moving forward again
                moveForward(NORMAL_SPEED);
            }
            break;

        default:
            break;
    }
}

void performTaskC() {
    if (currentMode != MODE_TASK_C) {
        return;  // Exit if not in Task C mode
    }
    Serial.println("Performing Task C...");
    // Implement Task C logic here
}

void handleExpressEmotion(AsyncWebServerRequest *request) {
    request->send(501, "text/plain", "Not Implemented");
}

void danceRoutine() {
    unsigned long currentMillis = millis();

    if (!danceInitiated) {
        // If dance has not been initiated, check space and start if enough room
        if (!checkSpaceForDance()) {
            Serial.println("Not enough space to dance :(");
            danceState = DANCE_COMPLETE; // Directly mark dance as complete if no space
            return;
        }
        Serial.println("Starting dance routine...");
        danceInitiated = true;
        danceState = DANCE_MOVE_FORWARD; // Start the first step of the dance
        danceStateStartTime = currentMillis;
    }

    switch (danceState) {
        case DANCE_MOVE_FORWARD:
            if (currentMillis - danceStateStartTime >= 1000) {
                spinLeft(NORMAL_SPEED);
                danceStateStartTime = currentMillis;
                danceState = DANCE_SPIN_LEFT;
            }
            break;

        case DANCE_SPIN_LEFT:
            if (currentMillis - danceStateStartTime >= 1000) {
                moveForward(NORMAL_SPEED);
                danceStateStartTime = currentMillis;
                danceState = DANCE_MOVE_FORWARD2;
            }
            break;

        case DANCE_MOVE_FORWARD2:
            if (currentMillis - danceStateStartTime >= 1000) {
                spinRight(NORMAL_SPEED);
                danceStateStartTime = currentMillis;
                danceState = DANCE_SPIN_RIGHT;
            }
            break;

        case DANCE_SPIN_RIGHT:
            if (currentMillis - danceStateStartTime >= 1000) {
                stopMotors();
                jiggleCount = 0;
                danceStateStartTime = currentMillis;
                danceState = DANCE_JIGGLE;
            }
            break;

        case DANCE_JIGGLE:
            if (jiggleCount < 5) {
                if (currentMillis - danceStateStartTime >= 400) {
                    moveForward(NORMAL_SPEED);
                    danceStateStartTime = currentMillis;
                    jiggleCount++;
                } else if (currentMillis - danceStateStartTime >= 200) {
                    moveBackward(NORMAL_SPEED);
                }
            } else {
                stopMotors();
                danceStateStartTime = currentMillis;
                danceState = DANCE_MOONWALK;
            }
            break;

        case DANCE_MOONWALK:
            if (currentMillis - danceStateStartTime >= 3000) {
                stopMotors();
                wiggleCount = 0;
                danceStateStartTime = currentMillis;
                danceState = DANCE_SPIN_WIGGLE;
            } else {
                moonwalkNonBlocking();
            }
            break;

        case DANCE_SPIN_WIGGLE:
            if (wiggleCount < 3) {
                if (currentMillis - danceStateStartTime >= 400) {
                    moveForward(NORMAL_SPEED);
                    danceStateStartTime = currentMillis;
                    wiggleCount++;
                } else if (currentMillis - danceStateStartTime >= 200) {
                    moveBackward(NORMAL_SPEED);
                }
            } else {
                stopMotors();
                danceState = DANCE_COMPLETE;
            }
            break;

        case DANCE_COMPLETE:
            Serial.println("Dance routine complete!");
            stopMotors();
            danceInitiated = false; // Reset to allow re-triggering of dance
            danceState = DANCE_IDLE; // Reset state
            break;

        default:
            break;
    }
}

bool checkSpaceForDance() {
    int distance = frontDistance;
    Serial.print("Front distance: ");
    Serial.println(distance);
    if (distance < 100) {
        Serial.println("Not enough space to dance :(");
        return false;
    }
    return true;
}

// Moonwalk non-blocking
void moonwalkNonBlocking() {
    static unsigned long moonwalkStartTime = millis();
    static bool moonwalkLeft = true;
    unsigned long currentMillis = millis();

    if (currentMillis - moonwalkStartTime >= 150) {
        moonwalkStartTime = currentMillis;
        if (moonwalkLeft) {
            spinLeft(NORMAL_SPEED);
            moonwalkLeft = false;
        } else {
            spinRight(NORMAL_SPEED);
            moonwalkLeft = true;
        }
    }
}

FaceOpenAreaState faceState = FACE_IDLE;
unsigned long faceStartTime = 0;

void faceOpenArea() {
    static unsigned long faceStartTime = 0; // Track when the state started
    static FaceOpenAreaState faceState = FACE_IDLE; // Default to idle

    unsigned long currentMillis = millis();

    switch (faceState) {
        case FACE_IDLE: {
            // Find the direction with the maximum open distance
            int maxDistance = max(frontDistance, max(rearDistance, max(leftDistance, rightDistance)));

            if (maxDistance == frontDistance) {
                Serial.println("Facing forward for the widest open area.");
                faceState = FACE_DONE; // Already facing forward
            } else if (maxDistance == rearDistance) {
                Serial.println("Turning to face backward for the widest open area.");
                float targetYaw = yaw + 180; // 180-degree turn
                if (targetYaw > 180) targetYaw -= 360; // Normalize yaw
                faceStartTime = currentMillis; // Start timer for timeout
                preciseTurn(targetYaw);
                faceState = FACE_TURNING_BACKWARD;
            } else if (maxDistance == leftDistance) {
                Serial.println("Turning to face left for the widest open area.");
                float targetYaw = yaw - 90; // 90-degree left turn
                if (targetYaw < -180) targetYaw += 360; // Normalize yaw
                faceStartTime = currentMillis; // Start timer for timeout
                preciseTurn(targetYaw);
                faceState = FACE_TURNING_LEFT;
            } else if (maxDistance == rightDistance) {
                Serial.println("Turning to face right for the widest open area.");
                float targetYaw = yaw + 90; // 90-degree right turn
                if (targetYaw > 180) targetYaw -= 360; // Normalize yaw
                faceStartTime = currentMillis; // Start timer for timeout
                preciseTurn(targetYaw);
                faceState = FACE_TURNING_RIGHT;
            }
            break;
        }

        case FACE_TURNING_BACKWARD:
            if (currentMillis - faceStartTime >= 2000) { // Timeout after 2 seconds
                stopMotors();
                Serial.println("Timeout reached while turning backward.");
                faceState = FACE_DONE; // Ensure the bot doesn't get stuck
            }
            break;

        case FACE_TURNING_LEFT:
            if (currentMillis - faceStartTime >= 1000) { // Timeout after 1 second
                stopMotors();
                Serial.println("Timeout reached while turning left.");
                faceState = FACE_DONE;
            }
            break;

        case FACE_TURNING_RIGHT:
            if (currentMillis - faceStartTime >= 1000) { // Timeout after 1 second
                stopMotors();
                Serial.println("Timeout reached while turning right.");
                faceState = FACE_DONE;
            }
            break;

        case FACE_DONE:
            Serial.println("Open area facing complete.");
            faceState = FACE_IDLE; // Reset state for future use
            break;
    }
}
void preciseTurn(float targetYaw, float speedPercentage) {
    // Ensure yaw is updated
    updateGyroData();

    // Calculate error between current yaw and target yaw
    float error = targetYaw - yaw;

    // Normalize error to be within -180 to 180 degrees
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;

    // Use proportional control to minimize error
    float Kp = 1.0; // Proportional gain (adjust as needed)
    float turnSpeed = Kp * error;

    // Limit turn speed to maximum allowed
    turnSpeed = constrain(turnSpeed, -speedPercentage, speedPercentage);

    if (abs(error) > 2.0) { // Continue turning if error is greater than 2 degrees
        if (turnSpeed > 0) {
            // Turn right
            spinRight(abs(turnSpeed));
        } else {
            // Turn left
            spinLeft(abs(turnSpeed));
        }
    } else {
        // Stop motors when the target is reached
        stopMotors();
    }
}



// Modified navigateToBeacon function with obstacle avoidance
void navigateToBeacon(int rssi) {
    Serial.printf("Beacon RSSI: %d\n", rssi);

    // Obstacle avoidance
    if (frontDistance < SAFE_DISTANCE) {
        Serial.println("Obstacle detected ahead, reacting...");
        reactState = REACT_START; // Initialize react state
        reactToCloseObstacleNonBlocking();
        return;
    }
    if (leftDistance < SAFE_DISTANCE) {
        Serial.println("Obstacle detected on the left, adjusting course...");
        spinRight(LOW_SPEED);
        return;
    }
    if (rightDistance < SAFE_DISTANCE) {
        Serial.println("Obstacle detected on the right, adjusting course...");
        spinLeft(LOW_SPEED);
        return;
    }

    if (rssi > followRSSIThreshold) {
        Serial.println("Beacon is close enough. Stopping.");
        stopMotors();
    } else if (rssi < -70) {
        Serial.println("Beacon far away, moving forward.");
        moveForward(NORMAL_SPEED);
    } else if (rssi >= -70 && rssi <= followRSSIThreshold) {
        Serial.println("Beacon nearby, fine-tuning position.");
        // Adjust direction based on side distances
        if (leftDistance > rightDistance) {
            spinLeft(LOW_SPEED);
        } else {
            spinRight(LOW_SPEED);
        }
    }
}

void followBeacon() {
    if (currentMode != MODE_FOLLOW_ME) {
        return;  // Exit if not in FOLLOW_ME mode
    }

    BLEScanResults results = pBLEScan->start(SCAN_TIME, false);
    bool beaconFound = false;
    int bestRSSI = -999; // Start with a very low RSSI

    for (int i = 0; i < results.getCount(); i++) {
        BLEAdvertisedDevice device = results.getDevice(i);
        if (device.haveServiceUUID() && String(device.getServiceUUID().toString().c_str()) == followBeaconUUID) {
            int rssi = device.getRSSI();
            Serial.printf("Found beacon: %s, RSSI: %d\n", device.getServiceUUID().toString().c_str(), rssi);
            beaconFound = true;
            bestRSSI = rssi; // Only care about this beacon
            break;
        }
    }

    if (beaconFound) {
        navigateToBeacon(bestRSSI); // Handle movement based on RSSI
    } else {
        Serial.println("Beacon not found. Stopping motors.");
        stopMotors();
    }
}

// ... (Other functions like danceRoutine and faceOpenArea remain unchanged)
