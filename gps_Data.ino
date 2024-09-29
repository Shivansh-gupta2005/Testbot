#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;  // Create a GPS object

// Teensy 4.1 has multiple UART ports; use Serial1 for GPS communication
#define gpsSerial Serial1  // Connect NEO-M8P-2's TX to Teensy's RX1 and RX to TX1

void setup() {
  // Start Serial communication for debugging
  Serial.begin(115200);
  while (!Serial);  // Wait for the serial connection to establish
  
  // Start UART communication with NEO-M8P-2
  gpsSerial.begin(9600);  // Set baud rate to 9600, which is the default for NEO-M8P-2
  
  Serial.println("NEO-M8P-2 GPS is starting...");
}

void loop() {
  // Check if GPS data is available
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);  // Feed the characters to the GPS object

    // Print raw GPS data if needed for debugging
    // Serial.print(c);
  }

  // Display the GPS data if it's ready
  if (gps.location.isUpdated()) {
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);  // Print latitude with 6 decimal places
    Serial.print(" Longitude: ");
    Serial.println(gps.location.lng(), 6);  // Print longitude with 6 decimal places

    Serial.print("Altitude: ");
    Serial.print(gps.altitude.meters());
    Serial.println(" m");

    Serial.print("Speed: ");
    Serial.print(gps.speed.kmph());
    Serial.println(" km/h");

    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
    
    Serial.print("HDOP: ");
    Serial.println(gps.hdop.value());
  }
}
