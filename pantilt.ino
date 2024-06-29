#include <WiFi.h>
#include <ESP32Servo.h>

const char* ssid = "SpectrumSetup-DD";
const char* password = "jeansrocket543";

Servo panServo;
Servo tiltServo;

const int panPin = 12;
const int tiltPin = 13;

int panPos = 90; // Start position facing forward
int tiltPos = 120; // Start position facing forward

WiFiServer server(80);

void setup() {
  Serial.begin(115200);

  panServo.attach(panPin);
  tiltServo.attach(tiltPin);

  // Set initial positions
  panServo.write(panPos);
  tiltServo.write(tiltPos);

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.begin();
  Serial.println("Server started.");
}

void loop() {
  WiFiClient client = server.available();

  if (client) {
    Serial.println("New client connected.");
    String request = client.readStringUntil('\r');
    Serial.print("Request: ");
    Serial.println(request);
    client.flush();

    if (request.indexOf("/zero") != -1) {
      panPos = 90;
      tiltPos = 120;
      panServo.write(panPos);
      tiltServo.write(tiltPos);
      Serial.println("Servos zeroed to 90 degrees (pan) and 120 degrees (tilt).");
    }

    if (request.indexOf("/panleft") != -1) {
      panPos = min(panPos + 10, 180);
      panServo.write(panPos);
      Serial.print("Pan left: ");
      Serial.println(panPos);
    }
    if (request.indexOf("/panright") != -1) {
      panPos = max(panPos - 10, 0);
      panServo.write(panPos);
      Serial.print("Pan right: ");
      Serial.println(panPos);
    }
    if (request.indexOf("/tiltup") != -1) {
      tiltPos = min(tiltPos + 10, 180); // Tilt up increases the angle
      tiltServo.write(tiltPos);
      Serial.print("Tilt up: ");
      Serial.println(tiltPos);
    }
    if (request.indexOf("/tiltdown") != -1) {
      tiltPos = max(tiltPos - 10, 90); // Tilt down decreases the angle
      tiltServo.write(tiltPos);
      Serial.print("Tilt down: ");
      Serial.println(tiltPos);
    }
    if (request.indexOf("/patrol") != -1) {
      Serial.println("Starting patrol...");
      performPatrol();
      Serial.println("Patrol finished.");
    }

    // Send the response to the client
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println("");
    client.println("<!DOCTYPE html>");
    client.println("<html lang='en'>");
    client.println("<head>");
    client.println("<meta charset='UTF-8'>");
    client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0'>");
    client.println("<title>ESP32 Servo Control</title>");
    client.println("<style>");
    client.println("body { display: flex; justify-content: center; align-items: center; height: 100vh; background: #e0e0e0; font-family: Arial, sans-serif; }");
    client.println(".container { display: flex; flex-direction: column; align-items: center; background: #e0e0e0; padding: 20px; border-radius: 20px; box-shadow: 20px 20px 60px #bebebe, -20px -20px 60px #ffffff; }");
    client.println(".button { width: 80px; height: 80px; margin: 10px; background: #e0e0e0; border-radius: 20px; display: flex; justify-content: center; align-items: center; box-shadow: 8px 8px 16px #bebebe, -8px -8px 16px #ffffff; cursor: pointer; font-size: 1.5em; color: #333; }");
    client.println(".button:active { box-shadow: inset 8px 8px 16px #bebebe, inset -8px -8px 16px #ffffff; }");
    client.println(".controls { display: flex; flex-direction: column; align-items: center; }");
    client.println(".pan-controls, .tilt-controls { display: flex; }");
    client.println("</style>");
    client.println("</head>");
    client.println("<body>");
    client.println("<div class='container'>");
    client.println("<h1>ESP32 Servo Control</h1>");
    client.println("<div class='controls'>");
    client.println("<div class='button' onclick='sendCommand(\"zero\")'>Reset</div>");
    client.println("<div class='tilt-controls'>");
    client.println("<div class='button' onclick='sendCommand(\"tiltup\")'>▲</div>");
    client.println("</div>");
    client.println("<div class='pan-controls'>");
    client.println("<div class='button' onclick='sendCommand(\"panleft\")'>◀</div>");
    client.println("<div class='button' onclick='sendCommand(\"panright\")'>▶</div>");
    client.println("</div>");
    client.println("<div class='tilt-controls'>");
    client.println("<div class='button' onclick='sendCommand(\"tiltdown\")'>▼</div>");
    client.println("</div>");
    client.println("<div class='button' onclick='sendCommand(\"patrol\")'>Patrol</div>");
    client.println("</div>");
    client.println("<div class='info'>Pan: <span id='panPos'>90</span>°</div>");
    client.println("<div class='info'>Tilt: <span id='tiltPos'>120</span>°</div>");
    client.println("</div>");
    client.println("<script>");
    client.println("const panPosElem = document.getElementById('panPos');");
    client.println("const tiltPosElem = document.getElementById('tiltPos');");
    client.println("function sendCommand(command) {");
    client.println("  fetch(`/${command}`).then(() => {");
    client.println("    if (command === 'zero') {");
    client.println("      panPosElem.textContent = 90;");
    client.println("      tiltPosElem.textContent = 120;");
    client.println("    }");
    client.println("    if (command === 'panleft') panPosElem.textContent = Math.min(parseInt(panPosElem.textContent) + 10, 180);");
    client.println("    if (command === 'panright') panPosElem.textContent = Math.max(parseInt(panPosElem.textContent) - 10, 0);");
    client.println("    if (command === 'tiltup') tiltPosElem.textContent = Math.min(parseInt(tiltPosElem.textContent) + 10, 180);"); // Tilt up increases the angle
    client.println("    if (command === 'tiltdown') tiltPosElem.textContent = Math.max(parseInt(tiltPosElem.textContent) - 10, 90);"); // Tilt down decreases the angle
    client.println("  });");
    client.println("}");
    client.println("</script>");
    client.println("</body>");
    client.println("</html>");

    delay(10);
    client.stop();
    Serial.println("Client disconnected.");
  }
}

void performPatrol() {
  // Look left to right
  for (int pos = 90; pos <= 180; pos += 5) {
    panServo.write(pos);
    delay(50);
  }
  for (int pos = 180; pos >= 0; pos -= 5) {
    panServo.write(pos);
    delay(50);
  }
  for (int pos = 0; pos <= 90; pos += 5) {
    panServo.write(pos);
    delay(50);
  }

  // Look up and down
  for (int pos = 120; pos <= 180; pos += 5) {
    tiltServo.write(pos);
    delay(50);
  }
  for (int pos = 180; pos >= 90; pos -= 5) {
    tiltServo.write(pos);
    delay(50);
  }
  for (int pos = 90; pos <= 120; pos += 5) {
    tiltServo.write(pos);
    delay(50);
  }
}
