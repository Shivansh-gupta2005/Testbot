#include <Wire.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

TinyGPSPlus gps;  // Create a GPS object

// Teensy 4.1 has multiple UART ports; use Serial1 for GPS communication
#define gpsSerial Serial1  // Connect NEO-M8P-2's TX to Teensy's RX1 and RX to TX1

// ROS Node Handle
ros::NodeHandle nh;

// ROS Publisher for NavSatFix
sensor_msgs::NavSatFix navsat_msg;
ros::Publisher navsat_pub("navsatfix", &navsat_msg);

void setup() {
    // Start Serial communication for debugging
    Serial.begin(115200);
    while (!Serial);  // Wait for the serial connection to establish

    // Start UART communication with NEO-M8P-2
    gpsSerial.begin(9600);  // Set baud rate to 9600, which is the default for NEO-M8P-2

    // Initialize ROS node
    nh.initNode();
    nh.advertise(navsat_pub);

    Serial.println("NEO-M8P-2 GPS is starting...");
}

void loop() {
    // Check if GPS data is available
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);  // Feed the characters to the GPS object
    }

    // Publish GPS data if it's ready
    if (gps.location.isUpdated()) {
        // Fill NavSatFix message with GPS data
        navsat_msg.latitude = gps.location.lat();
        navsat_msg.longitude = gps.location.lng();
        navsat_msg.altitude = gps.altitude.meters();
        
        // Set position covariance and its type (optional)
        navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

        // Fill status field with default values (as no fix info is provided by TinyGPS++)
        navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
        navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

        // Publish the NavSatFix message
        navsat_pub.publish(&navsat_msg);

        // Print GPS data to Serial Monitor for debugging
        Serial.print("Latitude: ");
        Serial.print(navsat_msg.latitude, 6);  
        Serial.print(" Longitude: ");
        Serial.println(navsat_msg.longitude, 6);

        Serial.print("Altitude: ");
        Serial.print(navsat_msg.altitude);
        Serial.println(" m");
    }

    nh.spinOnce();  // Process incoming ROS messages
}
