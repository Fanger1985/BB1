#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>  // For JSON processing

// Define LED RGB pin mappings
#define LED1_R 32
#define LED1_G 33
#define LED1_B 25

#define LED2_R 26
#define LED2_G 27
#define LED2_B 14

#define LED3_R 12
#define LED3_G 13
#define LED3_B 15

#define LED4_R 2
#define LED4_G 4
#define LED4_B 19

// WiFi credentials
const char* ssid = "SpectrumSetup-DD";  // Change to your WiFi SSID
const char* password = "jeansrocket543";  // Change to your WiFi password

// Mobile unit's IP address
const char* mobileUnitIP = "192.168.1.2";  // Change to your mobile unit's IP address

void setup() {
  Serial.begin(115200);  // Initialize Serial for debugging
  connectToWiFi();  // Connect to WiFi

  // Set LED pins as outputs
  setLEDsAsOutput();
}

void connectToWiFi() {
  WiFi.begin(ssid, password);  // Start connecting to WiFi
  Serial.println("Connecting to WiFi...");

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {  // Retry 20 times
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());  // Display IP address
  } else {
    Serial.println("Failed to connect to WiFi.");
  }
}

void loop() {
  requestMood();  // Check mood and set appropriate colors
  requestSensorData();  // Check sensor data for color changes
  delay(1000);  // Poll every second for frequent updates
}

void setLEDsAsOutput() {
  pinMode(LED1_R, OUTPUT);
  pinMode(LED1_G, OUTPUT);
  pinMode(LED1_B, OUTPUT);
  pinMode(LED2_R, OUTPUT);
  pinMode(LED2_G, OUTPUT);
  pinMode(LED2_B, OUTPUT);
  pinMode(LED3_R, OUTPUT);
  pinMode(LED3_G, OUTPUT);
  pinMode(LED3_B, OUTPUT);
  pinMode(LED4_R, OUTPUT);
  pinMode(LED4_G, OUTPUT);
  pinMode(LED4_B, OUTPUT);

  Serial.println("LED pins set as outputs.");
}

void requestSensorData() {
  HTTPClient http;
  http.begin("http://" + String(mobileUnitIP) + "/sensors");  // Endpoint to get sensor data
  int httpCode = http.GET();  // Send the HTTP GET request

  if (httpCode == 200) {  // If the request is successful
    String payload = http.getString();  // Get the response data
    Serial.println("Received sensor data:");
    Serial.println(payload);  // Display the raw data
    DynamicJsonDocument doc(256);  // For deserialization
    deserializeJson(doc, payload);

    int irLeft = doc["ir_left"];  // IR sensor left
    int irRight = doc["ir_right"];  // IR sensor right
    int distance = doc["distance"];  // Ultrasonic distance

    if (irLeft == 0) {
      Serial.println("Left rear IR sensor triggered. Setting to Orange.");
      setColorForAllLEDs(255, 165, 0);  // Orange
    } else if (irRight == 0) {
      Serial.println("Right rear IR sensor triggered. Setting to Red.");
      setColorForAllLEDs(255, 0, 0);  // Red
    } else if (distance < 30) {
      Serial.println("Ultrasonic sensor detected close proximity. Setting to Red.");
      setColorForAllLEDs(255, 0, 0);  // Red
    } else {
      Serial.println("All clear. Setting to Green.");
      setColorForAllLEDs(0, 255, 0);  // Green
    }
  } else {
    Serial.println("Failed to get sensor data. HTTP code: ");
    Serial.println(httpCode);  // Log the error code
  }

  http.end();  // End the HTTP session
}

void requestMood() {
  HTTPClient http;
  http.begin("http://" + String(mobileUnitIP) + "/expressEmotion");  // Endpoint for mood
  int httpCode = http.GET();  // Send HTTP GET request

  if (httpCode == 200) {  // If the request is successful
    String payload = http.getString();  // Get the response data
    Serial.println("Received mood data:");
    Serial.println(payload);  // Display the raw data
    DynamicJsonDocument doc(256);  // For deserialization
    deserializeJson(doc, payload);

    String emotion = doc["emotion"];  // Extract the mood or emotion

    if (emotion == "happy") {
      Serial.println("Setting color to Green for happy.");
      setColorForAllLEDs(0, 255, 0);  // Green for happy
    } else if (emotion == "sad") {
      Serial.println("Setting color to Blue for sad.");
      setColorForAllLEDs(0, 0, 255);  // Blue for sad
    } else if (emotion == "startled") {
      Serial.println("Setting color to Yellow for startled.");
      setColorForAllLEDs(255, 255, 0);  // Yellow for startled
    }

  } else {
    Serial.println("Failed to get mood data. HTTP code: ");
    Serial.println(httpCode);  // Log the error code
  }

  http.end();  // End the HTTP session
}

void setColorForAllLEDs(int r, int g, int b) {
  Serial.print("Setting all LEDs to RGB (");
  Serial.print(r);
  Serial.print(", ");
  Serial.print(g);
  Serial.print(", ");
  Serial.print(b);
  Serial.println(").");

  analogWrite(LED1_R, r);  // Set color for LED 1
  analogWrite(LED1_G, g);
  analogWrite(LED1_B, b);
  analogWrite(LED2_R, r);  // Set color for LED 2
  analogWrite(LED2_G, g);
  analogWrite(LED2_B, b);
  analogWrite(LED3_R, r);  // Set color for LED 3
  analogWrite(LED3_G, g);
  analogWrite(LED3_B, b);
  analogWrite(LED4_R, r);  // Set color for LED 4
  analogWrite(LED4_G, g);
  analogWrite(LED4_B, b);
}
