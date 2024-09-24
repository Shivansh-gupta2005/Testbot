#include <TinyGPS++.h>

// Create a TinyGPSPlus object to handle GPS data
TinyGPSPlus gps;

void setup() {
  // Start Serial for debugging (USB connection to Serial Monitor)
  Serial.begin(115200);
  while (!Serial) {
    // Wait for the Serial Monitor to open (only required for Teensy)
  }

  // Initialize Serial1 (Hardware Serial) for GPS communication
  Serial1.begin(9600);
  Serial.println("Serial1 initialized at 9600 baud for GPS");
}

void loop() {
  // Feed data from the GPS to TinyGPS++ by reading Serial1
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    Serial.write(c);  // Print raw NMEA data for debugging
    gps.encode(c);    // Feed data to TinyGPS++
  }

  // If valid location data is available, print it
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
  } else {
    Serial.println("Waiting for valid GPS data...");
  }

  // Print the number of satellites being tracked
  if (gps.satellites.isValid()) {
    Serial.print("Satellites: ");
    Serial.println(gps.satellites.value());
  } else {
    Serial.println("No valid satellite data.");
  }

  delay(1000);  // Wait for 1 second before reading the next data
}
