#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <MPU9250.h>
#include <Kalman.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <util/atomic.h>
#include <IntervalTimer.h>

// Task 1 variables (LCD)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Task 2 variables (MPU9250 + Kalman + ROS)
MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;
const float Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
float angleYaw, anglePitch, angleRoll;
ros::NodeHandle nh;
std_msgs::Float32 yaw_msg, pitch_msg, roll_msg;
ros::Publisher yaw_pub("yaw", &yaw_msg);
ros::Publisher pitch_pub("pitch", &pitch_msg);
ros::Publisher roll_pub("roll", &roll_msg);

// Task 3 variables (Encoders)
#define ENCA1 4 
#define ENCB1 5
#define ENCA2 10
#define ENCB2 11
#define ENCA3 35
#define ENCB3 34
#define ENCA4 36
#define ENCB4 37

std_msgs::Int32 pos_msg_1, pos_msg_2, pos_msg_3, pos_msg_4;
ros::Publisher pub_pos_1("pos_1", &pos_msg_1);
ros::Publisher pub_pos_2("pos_2", &pos_msg_2);
ros::Publisher pub_pos_3("pos_3", &pos_msg_3);
ros::Publisher pub_pos_4("pos_4", &pos_msg_4);

volatile int posi1 = 0, posi2 = 0, posi3 = 0, posi4 = 0;

// Task 4 variables (GPS)
TinyGPSPlus gps;  // Create a GPS object
#define gpsSerial Serial1  // Connect NEO-M8P-2's TX to Teensy's RX1 and RX to TX1
std_msgs::Float64 latitude_msg, longitude_msg, altitude_msg, speed_msg, hdop_msg;
std_msgs::Int32 satellites_msg;
ros::Publisher latitude_pub("gps/latitude", &latitude_msg);
ros::Publisher longitude_pub("gps/longitude", &longitude_msg);
ros::Publisher altitude_pub("gps/altitude", &altitude_msg);
ros::Publisher speed_pub("gps/speed", &speed_msg);
ros::Publisher satellites_pub("gps/satellites", &satellites_msg);
ros::Publisher hdop_pub("gps/hdop", &hdop_msg);

IntervalTimer myTimer;  // Timer object
int currentTask = 0;    // Task index

// Task 1: Read analog value and display on LCD
void task1() {
    int analogValue = analogRead(38);
    float calculatedValue = 22.2 / 1023 * analogValue;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Value: ");
    lcd.print(calculatedValue);
    Serial.print("Analog value: ");
    Serial.println(calculatedValue);
}

// Task 2: MPU9250 with Kalman filter and ROS publishing
void task2() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            float gyroX = mpu.getGyroX(), gyroY = mpu.getGyroY(), gyroZ = mpu.getGyroZ();
            float dt = 0.025;
            angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
            anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
            angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

            yaw_msg.data = angleYaw;
            pitch_msg.data = anglePitch;
            roll_msg.data = angleRoll;

            yaw_pub.publish(&yaw_msg);
            pitch_pub.publish(&pitch_msg);
            roll_pub.publish(&roll_msg);

            prev_ms = millis();
        }
    }
    nh.spinOnce();
}

// Task 3: Handle encoder positions and publish via ROS
void task3() {
    int pos1 = 0, pos2 = 0, pos3 = 0, pos4 = 0;

    // Atomic block to safely read volatile encoder positions
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        pos1 = posi1;
        pos2 = posi2;
        pos3 = posi3;
        pos4 = posi4;
    }

    // Publish encoder positions on respective ROS topics
    pos_msg_1.data = pos1;
    pub_pos_1.publish(&pos_msg_1);

    pos_msg_2.data = pos2;
    pub_pos_2.publish(&pos_msg_2);

    pos_msg_3.data = pos3;
    pub_pos_3.publish(&pos_msg_3);

    pos_msg_4.data = pos4;
    pub_pos_4.publish(&pos_msg_4);

    nh.spinOnce();
    delay(100);  // Delay for 100 ms
}

// Task 4: Read GPS data and publish via ROS
void task4() {
    // Check if GPS data is available
    while (gpsSerial.available() > 0) {
        char c = gpsSerial.read();
        gps.encode(c);  // Feed the characters to the GPS object
    }

    // Display the GPS data if it's updated
    if (gps.location.isUpdated()) {
        // Get the GPS data
        latitude_msg.data = gps.location.lat();
        longitude_msg.data = gps.location.lng();
        altitude_msg.data = gps.altitude.meters();
        speed_msg.data = gps.speed.kmph();
        satellites_msg.data = gps.satellites.value();
        hdop_msg.data = gps.hdop.value();

        // Publish the data
        latitude_pub.publish(&latitude_msg);
        longitude_pub.publish(&longitude_msg);
        altitude_pub.publish(&altitude_msg);
        speed_pub.publish(&speed_msg);
        satellites_pub.publish(&satellites_msg);
        hdop_pub.publish(&hdop_msg);

        // Print data for debugging
        Serial.print("Latitude: ");
        Serial.print(latitude_msg.data, 6);
        Serial.print(" Longitude: ");
        Serial.println(longitude_msg.data, 6);

        Serial.print("Altitude: ");
        Serial.print(altitude_msg.data);
        Serial.println(" m");

        Serial.print("Speed: ");
        Serial.print(speed_msg.data);
        Serial.println(" km/h");

        Serial.print("Satellites: ");
        Serial.println(satellites_msg.data);
        
        Serial.print("HDOP: ");
        Serial.println(hdop_msg.data);
    }

    // Spin the ROS node to process callbacks
    nh.spinOnce();
}

// ISR functions for each encoder
void readEncoder1() {
    int b = digitalRead(ENCB1);
    posi1 += (b > 0) ? 1 : -1;
}

void readEncoder2() {
    int b = digitalRead(ENCB2);
    posi2 += (b > 0) ? 1 : -1;
}

void readEncoder3() {
    int b = digitalRead(ENCB3);
    posi3 += (b > 0) ? 1 : -1;
}

void readEncoder4() {
    int b = digitalRead(ENCB4);
    posi4 += (b > 0) ? 1 : -1;
}

void setup() {
    // Setup for Task 1 (LCD)
    lcd.begin();
    lcd.backlight();
    pinMode(38, INPUT);

    // Setup for Task 2 (MPU9250 + ROS)
    Serial.begin(9600);
    Wire.begin();
    nh.initNode();
    nh.advertise(yaw_pub);
    nh.advertise(pitch_pub);
    nh.advertise(roll_pub);

    if (!mpu.setup(0x68)) {
        while (1) {
            nh.logerror("MPU connection failed.");
            delay(5000);
        }
    }

    kalmanYaw.setAngle(0); 
    kalmanPitch.setAngle(0); 
    kalmanRoll.setAngle(0);
    kalmanYaw.setQangle(Q_angle); 
    kalmanYaw.setQbias(Q_bias); 
    kalmanYaw.setRmeasure(R_measure);
    kalmanPitch.setQangle(Q_angle); 
    kalmanPitch.setQbias(Q_bias); 
    kalmanPitch.setRmeasure(R_measure);
    kalmanRoll.setQangle(Q_angle); 
    kalmanRoll.setQbias(Q_bias); 
    kalmanRoll.setRmeasure(R_measure);

    // Setup for Task 3 (Encoders)
    nh.advertise(pub_pos_1);
    nh.advertise(pub_pos_2);
    nh.advertise(pub_pos_3);
    nh.advertise(pub_pos_4);

    pinMode(ENCA1, INPUT);
    pinMode(ENCB1, INPUT);
    pinMode(ENCA2, INPUT);
    pinMode(ENCB2, INPUT);
    pinMode(ENCA3, INPUT);
    pinMode(ENCB3, INPUT);
    pinMode(ENCA4, INPUT);
    pinMode(ENCB4, INPUT);

    // Attach interrupts for encoders
    attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, CHANGE);

    // Setup for Task 4 (GPS)
    gpsSerial.begin(9600);  // Set baud rate to 9600 for the GPS
    nh.advertise(latitude_pub);
    nh.advertise(longitude_pub);
    nh.advertise(altitude_pub);
    nh.advertise(speed_pub);
    nh.advertise(satellites_pub);
    nh.advertise(hdop_pub);

    // Initialize the timer for task scheduling
    myTimer.begin([]() {
        currentTask = (currentTask + 1) % 4;  // Cycle through tasks 0 to 3
    }, 100000);  // Set timer to call every 100 ms (100,000 microseconds)

    Serial.println("Setup completed!");
}

void loop() {
    // Call the appropriate task based on the currentTask variable
    switch (currentTask) {
        case 0:
            task1();  // LCD Task
            break;
        case 1:
            task2();  // MPU9250 Task
            break;
        case 2:
            task3();  // Encoder Task
            break;
        case 3:
            task4();  // GPS Task
            break;
    }

    delay(10);  // Short delay to prevent overwhelming the loop
}

   
