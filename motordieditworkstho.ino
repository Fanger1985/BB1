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

// Ultrasonic Sensor 1
const int triggerPin1 = 16; // Trigger pin
const int echoPin1 = 17;   // Echo pin
// Ultrasonic Sensor 2
const int triggerPin2 = 4;  // Changed from 12 to 4 to avoid boot issues
const int echoPin2 = 13;    // Echo pin

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

// Function to manually control PWM
void customAnalogWrite(uint8_t pin, uint8_t value) { // Removed duration parameter for full speed
    digitalWrite(pin, value > 0 ? HIGH : LOW); // If value is greater than 0, set pin HIGH, otherwise LOW
}

// Setup function
void setup() {
    Serial.begin(115200);
  
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    pinMode(hallSensor1A, INPUT_PULLUP);
    pinMode(hallSensor1B, INPUT_PULLUP);
    pinMode(hallSensor2A, INPUT_PULLUP);
    pinMode(hallSensor2B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(hallSensor1A), hallSensor1AInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor1B), hallSensor1BInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor2A), hallSensor2AInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(hallSensor2B), hallSensor2BInterrupt, RISING);

    pinMode(triggerPin1, OUTPUT);
    pinMode(echoPin1, INPUT);
    pinMode(triggerPin2, OUTPUT);
    pinMode(echoPin2, INPUT);

    Serial.println("Setup complete. The robot is alive!");
}

// Assuming all previous code remains the same...
// Global constants for thresholds
const int obstacleThreshold = 30; // Threshold distance to consider something an obstacle
const int tooCloseThreshold = 10; // Threshold distance to consider stopping the bot

// Forward motion ramp-up function
void rampUpForward() {
  for (int speed = 0; speed <= 255; speed += 25) { // Ramp up speed gently
    customAnalogWrite(motor1Pin1, speed);
    customAnalogWrite(motor2Pin2, speed);
    delay(50); // Delay between speed increases to prevent tipping
  }
}
// Function to stop all motors
void stopMotors() {
    customAnalogWrite(motor1Pin1, 0); // Stop Motor 1
    customAnalogWrite(motor1Pin2, 0); // Stop Motor 1
    customAnalogWrite(motor2Pin1, 0); // Stop Motor 2
    customAnalogWrite(motor2Pin2, 0); // Stop Motor 2
    Serial.println("All motors stopped.");
}

// Function to back up the bot a little
void backUp() {
    customAnalogWrite(motor1Pin2, 255); // Motor 1 backward full speed
    customAnalogWrite(motor2Pin1, 255); // Motor 2 backward full speed
    delay(250); // Back up for 250 milliseconds
    stopMotors(); // Stop the motors after backing up
    Serial.println("Backing up...");
}
// Adjusted rampUpForward function to include speed parameter
void rampUpForward(int speed) {
  customAnalogWrite(motor1Pin1, speed); // Set Motor 1 forward to the specified speed
  customAnalogWrite(motor2Pin2, speed); // Set Motor 2 forward to the specified speed
  Serial.print("Ramping up to speed: ");
  Serial.println(speed);
}

// Function to execute a turn in place
void executeTurnInPlace(bool shouldTurnLeft) {
  if (shouldTurnLeft) {
    Serial.println("Executing in-place turn to the left");
    customAnalogWrite(motor1Pin2, 128); // Turn Motor 1 backward at half speed
    customAnalogWrite(motor2Pin2, 128); // Turn Motor 2 forward at half speed
  } else {
    Serial.println("Executing in-place turn to the right");
    customAnalogWrite(motor1Pin1, 128); // Turn Motor 1 forward at half speed
    customAnalogWrite(motor2Pin1, 128); // Turn Motor 2 backward at half speed
  }
  delay(500); // Duration of the turn
  stopMotors(); // Stop after turning
}

// Function to engage obstacle avoidance
void engageObstacleAvoidance(long distance1, long distance2) {
  Serial.println("Engaging obstacle avoidance");
  backUp(); // Back up first

  // Then decide the direction to turn based on the sensor distances
  bool shouldTurnLeft = distance1 > distance2;
  executeTurnInPlace(shouldTurnLeft);
}

// Function to execute random forward movement
void executeRandomForwardMovement() {
  int speed = random(100, 256); // Choose a random speed
  Serial.print("Executing random forward movement at speed: ");
  Serial.println(speed);
  rampUpForward(speed); // Move forward at the chosen speed
  delay(random(500, 1501)); // Continue for a random duration
}

void loop() {
  // Randomly decide on the bot's next action, including entering Scout Mode
  int action = random(0, 100);

  if (action < 5) {  // 5% chance to enter Scout Mode
    enterScoutMode();
  } else if (action < 15) {  // 10% chance to stop and "look around"
    stopMotors();
    Serial.println("Observing surroundings...");
    delay(random(500, 2001));  // Pause for 0.5 to 2 seconds
  } else if (action < 65) {  // 50% chance to move forward at a random speed
    int speed = random(100, 256);  // Choose a speed between 100 and 255
    rampUpForward(speed);
    Serial.print("Moving forward at speed: ");
    Serial.println(speed);
    delay(random(500, 1501));  // Continue for 0.5 to 1.5 seconds
  } else if (action < 85) {  // 20% chance to turn left or right in place
    bool shouldTurnLeft = random(2);  // Randomly choose true (left) or false (right)
    executeTurnInPlace(shouldTurnLeft);
  } else {  // 15% chance to engage in obstacle avoidance if needed
    long distance1 = readUltrasonicDistance(triggerPin1, echoPin1);
    long distance2 = readUltrasonicDistance(triggerPin2, echoPin2);

    if (distance1 < obstacleThreshold || distance2 < obstacleThreshold) {
      engageObstacleAvoidance(distance1, distance2);
    } else {
      executeRandomForwardMovement();
    }
  }

  // Short delay to prevent rapid oscillations
  delay(100);
}

void enterScoutMode() {
  Serial.println("Entering Scout Mode...");
  long maxDistance = 0;
  int bestDirection = 0;

  for (int i = 0; i < 3; i++) { // Spin in 3 full circles
    for (int angle = 0; angle < 360; angle += 30) { // Divide each circle into 12 sectors (every 30 degrees)
      executeTurnInPlace(true); // True for left, modify as needed for your bot
      delay(100); // Adjust based on your bot's turn speed to achieve a 30-degree turn

      long distance = readUltrasonicDistance(triggerPin1, echoPin1); // Use whichever ultrasonic sensor is forward-facing when the bot turns
      if (distance > maxDistance) {
        maxDistance = distance;
        bestDirection = angle; // Store the best direction (angle) with the maximum distance
      }
    }
  }

  // After spinning, turn to the best direction
  Serial.print("Best direction found at ");
  Serial.print(bestDirection);
  Serial.println(" degrees. Moving...");
  turnToBestDirection(bestDirection);

  // Move forward towards the open space
  rampUpForward(255); // Go full speed or adjust as needed
  delay(2000); // Move for 2 seconds or adjust based on your bot's speed and the space size
  stopMotors(); // Stop after moving to the open space
}

void turnToBestDirection(int bestDirection) {
  // Calculate how much and in which direction to turn to align with bestDirection
  // This is a simplified version, you might need to adjust the logic based on your bot's turning behavior
  executeTurnInPlace(true); // Assuming true is left, replace with correct direction
  delay((bestDirection / 30) * 100); // Adjust timing based on your bot's turn speed
  stopMotors();
}
void executeTurn(bool shouldTurnLeft, int duration) {
    if (shouldTurnLeft) {
        Serial.println("Turning left");
        customAnalogWrite(motor1Pin2, 0); 
        customAnalogWrite(motor1Pin1, 255); 
        customAnalogWrite(motor2Pin1, 255); 
        customAnalogWrite(motor2Pin2, 0); 
    } else {
        Serial.println("Turning right");
        customAnalogWrite(motor1Pin2, 255); 
        customAnalogWrite(motor1Pin1, 0); 
        customAnalogWrite(motor2Pin1, 0); 
        customAnalogWrite(motor2Pin2, 255); 
    }
    delay(duration); // Turn for the given duration
}


// Function to read distance from ultrasonic sensor
long readUltrasonicDistance(int triggerPin, int echoPin) {
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // Calculate distance

    return distance;
}
