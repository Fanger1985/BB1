#define NOTE_B0 31
#define NOTE_C1 33
#define NOTE_CS1 35
#define NOTE_D1 37
#define NOTE_DS1 39
#define NOTE_E1 41
#define NOTE_F1 44
#define NOTE_FS1 46
#define NOTE_G1 49
#define NOTE_GS1 52
#define NOTE_A1 55
#define NOTE_AS1 58
#define NOTE_B1 62
#define NOTE_C2 65
#define NOTE_CS2 69
#define NOTE_D2 73
#define NOTE_DS2 78
#define NOTE_E2 82
#define NOTE_F2 87
#define NOTE_FS2 93
#define NOTE_G2 98
#define NOTE_GS2 104
#define NOTE_A2 110
#define NOTE_AS2 117
#define NOTE_B2 123
#define NOTE_C3 131
#define NOTE_CS3 139
#define NOTE_D3 147
#define NOTE_DS3 156
#define NOTE_E3 165
#define NOTE_F3 175
#define NOTE_FS3 185
#define NOTE_G3 196
#define NOTE_GS3 208
#define NOTE_A3 220
#define NOTE_AS3 233
#define NOTE_B3 247
#define NOTE_C4 262
#define NOTE_CS4 277
#define NOTE_D4 294
#define NOTE_DS4 311
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_FS4 370
#define NOTE_G4 392
#define NOTE_GS4 415
#define NOTE_A4 440
#define NOTE_AS4 466
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_CS5 554
#define NOTE_D5 587
#define NOTE_DS5 622
#define NOTE_E5 659
#define NOTE_F5 698
#define NOTE_FS5 740
#define NOTE_G5 784
#define NOTE_GS5 831
#define NOTE_A5 880
#define NOTE_AS5 932
#define NOTE_B5 988
#define NOTE_C6 1047
#define NOTE_CS6 1109
#define NOTE_D6 1175
#define NOTE_DS6 1245
#define NOTE_E6 1319
#define NOTE_F6 1397
#define NOTE_FS6 1480
#define NOTE_G6 1568
#define NOTE_GS6 1661
#define NOTE_A6 1760
#define NOTE_AS6 1865
#define NOTE_B6 1976
#define NOTE_C7 2093
#define NOTE_CS7 2217
#define NOTE_D7 2349
#define NOTE_DS7 2489
#define NOTE_E7 2637
#define NOTE_F7 2794
#define NOTE_FS7 2960
#define NOTE_G7 3136
#define NOTE_GS7 3322
#define NOTE_A7 3520
#define NOTE_AS7 3729
#define NOTE_B7 3951
#define NOTE_C8 4186
#define NOTE_CS8 4435
#define NOTE_D8 4699
#define NOTE_DS8 4978

#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";
const char* serverIP = "192.168.1.2"; // Change this to your robot's server IP

// Define the pin the buzzer is attached to
int buzzerPin = 23;

// Expanded Mario melody
int marioMelody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};

// Expanded Mario durations
int marioNoteDurations[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  9, 9, 9,
  12, 12, 12, 12,
  12, 9, 9, 9,
  12, 12, 12, 12, 12,
  12, 12, 12, 12
};

// Tetris Theme (Korobeiniki) melody corrected for rests
int tetrisMelody[] = {
  NOTE_E5, NOTE_B4, NOTE_C5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_A4,
  NOTE_A4, NOTE_C5, NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_B4,
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4, 0,
  NOTE_D5, NOTE_F5, NOTE_A5, NOTE_G5, NOTE_F5, NOTE_E5, NOTE_C5,
  NOTE_E5, NOTE_D5, NOTE_C5, NOTE_B4, NOTE_B4, NOTE_C5, NOTE_D5,
  NOTE_E5, NOTE_C5, NOTE_A4, NOTE_A4
};

// Note durations for Tetris Theme (Korobeiniki)
int tetrisNoteDurations[] = {
  8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 16, // Rest length adjusted here if needed
  8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8, 8, 8, 8,
  8, 8, 8, 8
};

// Mario Pipe Sound effect
int marioPipeSound[] = {
  NOTE_G4, NOTE_F4, NOTE_E4, NOTE_C4, NOTE_A3
};

// Note durations for Mario Pipe Sound
int marioPipeNoteDurations[] = {
  10, 10, 10, 10, 10  // very quick succession to mimic the sliding effect
};


// Super Mario Bros. Main Theme melody
int superMarioMelody[] = {
  NOTE_E5, NOTE_E5, 0, NOTE_E5,
  0, NOTE_C5, NOTE_E5, 0,
  NOTE_G5, 0, 0,  0,
  NOTE_G4, 0, 0, 0,

  NOTE_C5, 0, 0, NOTE_G4,
  0, 0, NOTE_E4, 0,
  0, NOTE_A4, 0, NOTE_B4,
  0, NOTE_AS4, NOTE_A4, 0,

  NOTE_G4, NOTE_E5, NOTE_G5,
  NOTE_A5, 0, NOTE_F5, NOTE_G5,
  0, NOTE_E5, 0, NOTE_C5,
  NOTE_D5, NOTE_B4, 0, 0
};

// Super Mario Bros. Main Theme note durations
int superMarioNoteDurations[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,

  9, 9, 9,
  12, 12, 12, 12,
  12, 9, 9, 9,
  12, 12, 12, 12
};

// We’ll wrap the playing into functions to keep it organized:
void playMelody(int buzzerPin, int melody[], int noteDurations[], int length) {
  for (int thisNote = 0; thisNote < length; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    if (melody[thisNote] == 0) {
      noTone(buzzerPin); // Stop playing any tone
    } else {
      tone(buzzerPin, melody[thisNote], noteDuration); // Play the note
    }
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(buzzerPin); // Ensure the tone is stopped before the next note
  }
}

void setup() {
    Serial.begin(9600); // Start serial communication at 9600 baud
    pinMode(buzzerPin, OUTPUT);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nConnected to WiFi.");
}

// Function to fetch sensor data from the robot
DynamicJsonDocument fetchSensorData() {
    HTTPClient http;
    DynamicJsonDocument doc(1024); // Adjust size based on your JSON data
String serverUrl = "http://" + String(serverIP) + "/sensors"; // Make sure to add "/sensors" at the end however that stops it from working currently

    http.begin(serverUrl);  // URL to the server's sensor data endpoint
    Serial.println("Attempting to fetch data from: " + serverUrl); // Log the URL being hit

    int httpCode = http.GET();
    if (httpCode == 200) {
        String payload = http.getString();
        deserializeJson(doc, payload);
        Serial.println("Data fetched successfully!");
        Serial.println("Payload: " + payload);  // Log the JSON payload received
    } else {
        Serial.print("Failed to fetch data, HTTP code: ");
        Serial.println(httpCode);  // Log the HTTP error code
    }
    http.end();
    return doc;
}

// Functions to play melodies based on sensor data
void playSoundBasedOnDistance(int distance) {
  if (distance < 30) { // Close object
    playMelody(buzzerPin, marioPipeSound, marioPipeNoteDurations, sizeof(marioPipeSound) / sizeof(int));
  } else if (distance >= 30 && distance < 100) { // Moderate distance
    playMelody(buzzerPin, tetrisMelody, tetrisNoteDurations, sizeof(tetrisMelody) / sizeof(int));
  } else { // Far or no object
    playMelody(buzzerPin, superMarioMelody, superMarioNoteDurations, sizeof(superMarioMelody) / sizeof(superMarioMelody[0]));
  }
}
// Assuming IR sensor values and distance are already fetched and stored in variables
void playSoundsBasedOnSensors(int distance, int irLeft, int irRight) {
    // Check IR sensors first
    if (irLeft == LOW || irRight == LOW) {  // Assuming LOW means obstacle detected
        Serial.println("Obstacle detected by IR sensors!");
        playMelody(buzzerPin, marioPipeSound, marioPipeNoteDurations, sizeof(marioPipeSound) / sizeof(int));
    } else {
        // Now check the ultrasonic sensor
        if (distance < 30) {
            Serial.println("Playing alert sound for close objects...");
            playMelody(buzzerPin, tetrisMelody, tetrisNoteDurations, sizeof(tetrisMelody) / sizeof(int));
        } else if (distance >= 30 && distance < 100) {
            Serial.println("Playing moderate distance sound...");
            playMelody(buzzerPin, superMarioMelody, superMarioNoteDurations, sizeof(superMarioMelody) / sizeof(superMarioMelody[0]));
        } else {
            Serial.println("No nearby obstacles. Playing happy tune...");
            playMelody(buzzerPin, marioMelody, marioNoteDurations, sizeof(marioMelody) / sizeof(int));
        }
    }
}

void loop() {
    Serial.println("Fetching sensor data...");
    DynamicJsonDocument sensorData = fetchSensorData();

    if (!sensorData.isNull()) {
        int distance = sensorData["distance"];
        int irLeft = sensorData["ir_left"];
        int irRight = sensorData["ir_right"];

        Serial.print("Distance: ");
        Serial.println(distance);
        Serial.print("IR Left: ");
        Serial.println(irLeft);
        Serial.print("IR Right: ");
        Serial.println(irRight);

        playSoundsBasedOnSensors(distance, irLeft, irRight);
    } else {
        Serial.println("No data received, check connection or server status.");
    }
    delay(1000); // Delay next fetch/play cycle
}
