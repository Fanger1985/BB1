#include <Arduino.h>

// Motor 1
const int motor1Pin1 = 5;  // IN1 on the DRV8871
const int motor1Pin2 = 18; // IN2 on the DRV8871
// Motor 2
const int motor2Pin1 = 19; // IN1 on the DRV8871 
const int motor2Pin2 = 21; // IN2 on the DRV8871

// Hall sensors
const int hallSensor1A = 34; // Hall sensor A for motor 1 right from back
const int hallSensor1B = 35; // Hall sensor B for motor 1
const int hallSensor2A = 32; // Hall sensor A for motor 2 left from back
const int hallSensor2B = 33; // Hall sensor B for motor 2

// Hall sensor counters
volatile int hallCount1A = 0;
volatile int hallCount1B = 0;
volatile int hallCount2A = 0;
volatile int hallCount2B = 0;

// Ultrasonic Sensor for front distance measurement
const int triggerPin1 = 16; // Trigger pin
const int echoPin1 = 17;   // Echo pin
// Rear IR sensors
const int rearRightIRSensor = 4; // Rear right IR sensor (5 o'clock)
const int rearLeftIRSensor = 13; // Rear left IR sensor (7 o'clock)

// ISR for hall sensors
void IRAM_ATTR hallSensor1AInterrupt() {
  hallCount1A++;
}

void IRAM_ATTR hallSensor1BInterrupt() {
  hallCount1B++;
}

void IRAM_ATTR hallSensor2AInterrupt() {
  hallCount2A++;
}

void IRAM_ATTR hallSensor2BInterrupt() {
  hallCount2B++;
}

void setMotorDirection(int motorPin1, int motorPin2, bool forward) {
    digitalWrite(motorPin1, forward ? HIGH : LOW);
    digitalWrite(motorPin2, forward ? LOW : HIGH);
}

void setup() {
    Serial.begin(115200);

    // Initialize motor pins as outputs
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    // Initialize hall sensor pins as inputs with pullup resistors
    pinMode(hallSensor1A, INPUT_PULLUP);
    pinMode(hallSensor1B, INPUT_PULLUP);
    pinMode(hallSensor2A, INPUT_PULLUP);
    pinMode(hallSensor2B, INPUT_PULLUP);

    // Attach interrupts to hall sensors for counting rotations
    attachInterrupt(digitalPinToInterrupt(hallSensor1A), hallSensor1AInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor1B), hallSensor1BInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor2A), hallSensor2AInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor2B), hallSensor2BInterrupt, RISING);

    // Initialize front ultrasonic sensor pins
    pinMode(triggerPin1, OUTPUT);
    pinMode(echoPin1, INPUT);

    // Initialize rear IR sensors as inputs
    pinMode(rearRightIRSensor, INPUT);
    pinMode(rearLeftIRSensor, INPUT);

    Serial.println("Setup complete. The bot's ready to roll out!");
}

// Global constants for thresholds
const int obstacleThreshold = 40; // Threshold distance to consider something an obstacle
const int tooCloseThreshold = 20; // Threshold distance to consider stopping the bot

void stopMotors() {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
    Serial.println("All motors stopped.");
}

void rampUpForward(int speed) {
    analogWrite(motor1Pin1, speed); // Set speed for Motor 1 forward
    digitalWrite(motor1Pin2, LOW);  // Ensure Motor 1 moves forward

    analogWrite(motor2Pin1, speed); // Set speed for Motor 2 forward
    digitalWrite(motor2Pin2, LOW);  // Ensure Motor 2 moves forward
}


void backUp() {
    setMotorDirection(motor1Pin2, motor1Pin1, false); // Motor 1 backward
    setMotorDirection(motor2Pin1, motor2Pin2, false); // Motor 2 backward
    delay(250); // Back up for 250 milliseconds
    stopMotors();
    Serial.println("Backing up...");
}

void executeTurnInPlace(bool shouldTurnLeft) {
    if (shouldTurnLeft) {
        Serial.println("Executing in-place turn to the left");
        setMotorDirection(motor1Pin1, motor1Pin2, true); // Turn Motor 1 forward
        setMotorDirection(motor2Pin1, motor2Pin2, false); // Turn Motor 2 backward
        delay(500); // Duration of the turn
    } else {
        Serial.println("Executing in-place turn to the right");
        setMotorDirection(motor1Pin1, motor1Pin2, false); // Turn Motor 1 backward
        setMotorDirection(motor2Pin2, motor2Pin1, true); // Turn Motor 2 forward
        delay(550); // Slightly longer duration for right turn to compensate
    }
    stopMotors(); // Stop after turning
}


long readUltrasonicDistanceInInches(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distanceCm = duration * 0.034 / 2; // Convert time to distance
    return distanceCm / 2.54; // Convert cm to inches
}
void resetUltrasonicSensor() {
    pinMode(triggerPin1, INPUT); // Turn off trigger pin
    delay(10); // Wait for 10 milliseconds
    pinMode(triggerPin1, OUTPUT); // Turn trigger pin back on
}

// IR sensor debounce delay in milliseconds
const unsigned long debounceDelay = 50; 
// IR sensor detection threshold (milliseconds of consistent detection to take action)
const unsigned long detectionThreshold = 200; 
// Last time IR sensors detected something
unsigned long lastRearRightDetectionTime = 0;
unsigned long lastRearLeftDetectionTime = 0;

bool isObstacleBehind() {
    unsigned long currentMillis = millis();
    bool rearRight = digitalRead(rearRightIRSensor) == LOW; // Changed to LOW for active low sensors
    bool rearLeft = digitalRead(rearLeftIRSensor) == LOW;   // Changed to LOW for active low sensors

    // Check if the right sensor has been consistently detecting something
    if (rearRight) {
        if (currentMillis - lastRearRightDetectionTime > detectionThreshold) {
            Serial.println("Confirmed obstacle on the rear right!");
            lastRearRightDetectionTime = currentMillis; // Reset the detection time
            return true;
        }
    } else {
        lastRearRightDetectionTime = currentMillis; // Reset if no detection
    }

    // Check if the left sensor has been consistently detecting something
    if (rearLeft) {
        if (currentMillis - lastRearLeftDetectionTime > detectionThreshold) {
            Serial.println("Confirmed obstacle on the rear left!");
            lastRearLeftDetectionTime = currentMillis; // Reset the detection time
            return true;
        }
    } else {
        lastRearLeftDetectionTime = currentMillis; // Reset if no detection
    }

    delay(debounceDelay); 
    return false; // No consistent obstacle detected
}


void evadeRearObstacle() {
    // Get the current time for comparison
    unsigned long currentMillis = millis();

    // Check the rear right sensor
    if (digitalRead(rearRightIRSensor) == HIGH) {
        // If the sensor is still triggered after the threshold time, it's a real obstacle
        if (currentMillis - lastRearRightDetectionTime > detectionThreshold) {
            Serial.println("Real spooky stuff behind on the right, swinging left!");
            executeTurnInPlace(true); // Turn in place to the left
            lastRearRightDetectionTime = currentMillis; // Reset the detection time
        }
    } else {
        lastRearRightDetectionTime = currentMillis; // Reset if no longer detected
    }

    // Check the rear left sensor
    if (digitalRead(rearLeftIRSensor) == HIGH) {
        // If the sensor is still triggered after the threshold time, it's a real obstacle
        if (currentMillis - lastRearLeftDetectionTime > detectionThreshold) {
            Serial.println("Real spooky stuff behind on the left, swinging right!");
            executeTurnInPlace(false); // Turn in place to the right
            lastRearLeftDetectionTime = currentMillis; // Reset the detection time
        }
    } else {
        lastRearLeftDetectionTime = currentMillis; // Reset if no longer detected
    }
}


bool lastTurnWasLeft = false; // Initialize the last turn direction as right

void turnRight() {
    Serial.println("Turning right");
    digitalWrite(motor1Pin1, LOW); // Left tread forward
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH); // Right tread backward
    digitalWrite(motor2Pin2, LOW);
    delay(500); // Adjust the delay to control the turn's sharpness
    stopMotors();
}
int scanForBestPath() {
    Serial.println("Scanning for the best path...");
    long bestDistance = 0;
    int bestDirection = 0; // 0 for forward, -1 for left, 1 for right

    // Reduce motor speed for scanning turns to make them smoother
    int originalSpeed = 128; // Adjust as needed
    analogWrite(motor1Pin1, originalSpeed / 2); // Reduce speed
    analogWrite(motor2Pin1, originalSpeed / 2); // Reduce speed

    // Scan left
    executeTurnInPlace(true);
    delay(700); // Increased delay to ensure the turn completes
    long leftDistance = readUltrasonicDistanceInInches(triggerPin1, echoPin1);

    // Scan right (by turning right twice the angle to cover the original position plus right scan)
    executeTurnInPlace(false);
    delay(1400); // Double the delay for a complete right scan
    long rightDistance = readUltrasonicDistanceInInches(triggerPin1, echoPin1);

    // Return to original orientation by turning left
    executeTurnInPlace(true);
    delay(700); // Return to the original direction

    // Restore original motor speed after scanning
    analogWrite(motor1Pin1, originalSpeed);
    analogWrite(motor2Pin1, originalSpeed);

    // Determine the best direction based on the scanned distances
    if (leftDistance > rightDistance) {
        bestDistance = leftDistance;
        bestDirection = -1; // Left is best
    } else {
        bestDistance = rightDistance;
        bestDirection = 1; // Right is best
    }

    // If neither direction offers a better path, the best direction remains 0 (forward)
    if (bestDistance <= obstacleThreshold) {
        bestDirection = 0;
    }

    Serial.print("Best direction determined: ");
    Serial.println(bestDirection == 0 ? "Forward" : (bestDirection == -1 ? "Left" : "Right"));

    return bestDirection;
}

void backUpAndScan() {
    Serial.println("Too close! Backing up...");
    backUp(); // Back up a little to give some space
    
    // Now let's do a full 360 turn, scanning as we go
    Serial.println("Scanning for a way out...");
    long bestDistance = 0;
    int bestAngle = 0;
    for (int angle = 0; angle < 360; angle += 45) { // Divide the circle into 45-degree increments
        executeTurnInPlace(true); // Start turning
        delay(100); // Wait for a bit to complete the turn increment
        long distance = readUltrasonicDistanceInInches(triggerPin1, echoPin1);
        if (distance > bestDistance) { // We're looking for the largest clear space
            bestDistance = distance;
            bestAngle = angle;
        }
    }

    // Rotate to the best angle found
    Serial.print("Best path found at ");
    Serial.print(bestAngle);
    Serial.println(" degrees, heading that way!");
    executeTurnInPlace(bestAngle <= 180); // Turn the shortest angle towards the best path
    delay((abs(180 - bestAngle) / 180) * 500); // Turn for a time proportional to the angle
    stopMotors(); // And stop to proceed with that direction
}

// Global variable to count loop iterations for ultrasonic sensor reset
int ultrasonicResetCounter = 0;

// Global variables for timing
unsigned long lastMotorUpdate = 0;
unsigned long lastUltrasonicReset = 0;
const unsigned long motorUpdateInterval = 1500; // Interval for motor actions
const unsigned long ultrasonicResetInterval = 30000; // Interval to reset the ultrasonic sensor, e.g., every 30 seconds

unsigned long lastUpdate = 0;
const unsigned long updateInterval = 2000; // Time in milliseconds for movement updates

unsigned long lastSensorCheck = 0;
const unsigned long sensorCheckInterval = 100; // Time in milliseconds for sensor checks

unsigned long lastSensorMaintenance = 0;
const unsigned long sensorMaintenanceInterval = 30000; // Time in milliseconds for sensor maintenance

void loop() {
    unsigned long currentMillis = millis();

    // Periodic sensor maintenance
    if (currentMillis - lastSensorMaintenance >= sensorMaintenanceInterval) {
        lastSensorMaintenance = currentMillis;
        resetUltrasonicSensor(); // Reset the ultrasonic sensor
        // Add any other sensor maintenance tasks here
        Serial.println("Sensor maintenance complete.");
    }

    // Check sensors at regular intervals, not every loop cycle
    if (currentMillis - lastSensorCheck >= sensorCheckInterval) {
        lastSensorCheck = currentMillis;

        // Check for obstacles behind
        if (isObstacleBehind()) {
            evadeRearObstacle();
            // Reset the motors after evading rear obstacles to prevent immediate forward movement
            stopMotors();
            delay(500); // Give a brief pause after the action
        } else {
            // Front distance check
            long frontDistance = readUltrasonicDistanceInInches(triggerPin1, echoPin1);
            Serial.print("Front Distance: ");
            Serial.println(frontDistance);

            // Time to move based on the front distance
            if (currentMillis - lastUpdate >= updateInterval) {
                lastUpdate = currentMillis; // Update the last update time
                
                if (frontDistance > obstacleThreshold) {
                    Serial.println("Path is clear. Moving forward.");
                    rampUpForward(255); // Move forward at full speed
                } else {
                    Serial.println("Obstacle detected. Scanning for the best path...");
                    int bestDirection = scanForBestPath();

                    stopMotors(); // Stop and stabilize before taking action
                    delay(200); // Brief pause to stabilize

                    // Decide on the movement based on the best direction determined
                    if (bestDirection == -1) { // If left is the best direction
                        Serial.println("Turning left.");
                        executeTurnInPlace(true); // Turn left
                        delay(500); // Wait for the turn to complete
                    } else if (bestDirection == 1) { // If right is the best direction
                        Serial.println("Turning right.");
                        executeTurnInPlace(false); // Turn right
                        delay(500); // Wait for the turn to complete
                    }

                    // Gradually ramp up speed to move in the chosen direction
                    for (int speed = 0; speed <= 255; speed += 25) {
                        rampUpForward(speed); // Adjust speed gradually
                        delay(50); // Short delay for a smooth ramp-up
                    }
                }
            }
        }
    }

    // No need for a delay at the end of the loop, keeping the bot's actions smooth and responsive.
}
