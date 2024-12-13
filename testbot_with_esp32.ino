#include <WiFi.h>
#include <WebServer.h>

// Motor pin definitions
#define DIR1 2   //8
#define PWM1 18  //9
#define DIR2 4   //6
#define PWM2 5   //7
#define DIR3 19 //15
#define PWM3 14  //14
#define DIR4 27  //22
#define PWM4 15  //20

// WiFi credentials - replace with your network details
const char* ssid = "Nokia G42 5G";
const char* password = "shivansh2005";

WebServer server(80);

// HTML webpage
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        .button {
            background-color: #4CAF50;
            border: none;
            color: white;
            padding: 16px 40px;
            text-align: center;
            font-size: 16px;
            margin: 4px 2px;
            cursor: pointer;
        }
        .container {
            text-align: center;
            padding: 20px;
        }
        .controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 0 auto;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Robot Control</h1>
        <div class="controls">
            <div></div>
            <button class="button" onmousedown="sendCommand('forward')" onmouseup="sendCommand('stop')">Forward</button>
            <div></div>
            
            <button class="button" onmousedown="sendCommand('left')" onmouseup="sendCommand('stop')">Left</button>
            <button class="button" onmousedown="sendCommand('stop')" onmouseup="sendCommand('stop')">Stop</button>
            <button class="button" onmousedown="sendCommand('right')" onmouseup="sendCommand('stop')">Right</button>
            
            <div></div>
            <button class="button" onmousedown="sendCommand('backward')" onmouseup="sendCommand('stop')">Backward</button>
            <div></div>
            
            <button class="button" onmousedown="sendCommand('spinleft')" onmouseup="sendCommand('stop')">Spin Left</button>
            <div></div>
            <button class="button" onmousedown="sendCommand('spinright')" onmouseup="sendCommand('stop')">Spin Right</button>
        </div>
    </div>
    <script>
        function sendCommand(command) {
            var xhr = new XMLHttpRequest();
            xhr.open("GET", "/"+command, true);
            xhr.send();
        }
    </script>
</body>
</html>
)rawliteral";

void setup() {
    Serial.begin(115200);
    
    // Set pin modes
    pinMode(DIR1, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(DIR3, OUTPUT);
    pinMode(DIR4, OUTPUT);
    pinMode(PWM1, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(PWM4, OUTPUT);
    
    // Initialize all pins to LOW
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, 0);
    
    // Connect to WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println(WiFi.localIP());
    
    // Setup web server routes
    server.on("/", HTTP_GET, handleRoot);
    server.on("/forward", HTTP_GET, moveForward);
    server.on("/backward", HTTP_GET, moveBackward);
    server.on("/left", HTTP_GET, moveLeft);
    server.on("/right", HTTP_GET, moveRight);
    server.on("/spinleft", HTTP_GET, spinLeft);
    server.on("/spinright", HTTP_GET, spinRight);
    server.on("/stop", HTTP_GET, stopMotors);
    
    server.begin();
}

void loop() {
    server.handleClient();
}

void handleRoot() {
    server.send(200, "text/html", index_html);
}

void moveForward() {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 255);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 255);
    analogWrite(PWM4, 0);
    server.send(200, "text/plain", "Moving forward");
}

void moveLeft() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 255);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, 255);
    server.send(200, "text/plain", "Moving left");
}

void moveBackward() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, HIGH);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 255);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 255);
    analogWrite(PWM4, 0);
    server.send(200, "text/plain", "Moving backward");
}

void moveRight() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 255);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, 255);
    server.send(200, "text/plain", "Moving right");
}

void spinLeft() {
    digitalWrite(DIR1, LOW);
    digitalWrite(DIR2, HIGH);
    digitalWrite(DIR3, LOW);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM1, 255);
    analogWrite(PWM2, 255);
    analogWrite(PWM3, 255);
    analogWrite(PWM4, 255);
    server.send(200, "text/plain", "Spinning left");
}

void spinRight() {
    digitalWrite(DIR1, HIGH);
    digitalWrite(DIR2, LOW);
    digitalWrite(DIR3, HIGH);
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM1, 255);
    analogWrite(PWM2, 255);
    analogWrite(PWM3, 255);
    analogWrite(PWM4, 255);
    server.send(200, "text/plain", "Spinning right");
}

void stopMotors() {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, 0);
    server.send(200, "text/plain", "Stopped");
}
