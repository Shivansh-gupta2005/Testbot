#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Define the pins for RX and TX
const int RX_PIN = 28;  // Connect to TX of GPS module
const int TX_PIN = 29;  // Connect to RX of GPS module

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);     // Serial communication with the computer
  gpsSerial.begin(9600);  // Serial communication with the GPS module
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);
      } else {
        Serial.println("Location: Not Available");
      }
      
      Serial.println(); // Add a blank line for readability
    }
  }

  // If no data is received for a long time, print an error
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS detected: check wiring.");
    delay(1000);
  }
}
