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

#define FADE_TIME 1000  // Time in milliseconds to fade in or out

void setup() {
  // Initiate the RGB LED pins as outputs
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
}

void loop() {
  // Loop through each color (R, G, B)
  for (int color = 0; color < 3; color++) {
    // Fade in
    for (int brightness = 0; brightness <= 255; brightness++) {
      setAllLEDs(color, brightness);
      delay(FADE_TIME / 255);
    }
    // Fade out
    for (int brightness = 255; brightness >= 0; brightness--) {
      setAllLEDs(color, brightness);
      delay(FADE_TIME / 255);
    }
  }
}

void setAllLEDs(int color, int brightness) {
  // Apply the brightness to all LEDs based on the color
  for (int led = 1; led <= 4; led++) {
    setLED(led, color, brightness);
  }
}

void setLED(int led, int color, int brightness) {
  int redPin, greenPin, bluePin;
  // Assign the pins to each LED
  switch (led) {
    case 1:
      redPin = LED1_R; greenPin = LED1_G; bluePin = LED1_B;
      break;
    case 2:
      redPin = LED2_R; greenPin = LED2_G; bluePin = LED2_B;
      break;
    case 3:
      redPin = LED3_R; greenPin = LED3_G; bluePin = LED3_B;
      break;
    case 4:
      redPin = LED4_R; greenPin = LED4_G; bluePin = LED4_B;
      break;
  }

  // Activate the color for the specific LED
  if (color == 0) {
    analogWrite(redPin, brightness);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
  } else if (color == 1) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, brightness);
    analogWrite(bluePin, 0);
  } else if (color == 2) {
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, brightness);
  }
}
