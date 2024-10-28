#include <Wire.h>
#include <MPU9250.h>
#include <Kalman.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <std_msgs/Int32MultiArray.h>
#include <IntervalTimer.h>

// IMU Setup
MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;
const float Q_angle = 0.001;   // Process noise covariance for angle
const float Q_bias = 0.003;    // Process noise covariance for bias
const float R_measure = 0.03;  // Measurement noise covariance

float angleYaw, anglePitch, angleRoll;

// GPS Setup
TinyGPSPlus gps;  // Create a GPS object
#define gpsSerial Serial1  // Connect NEO-M8P-2's TX to Teensy's RX1 and RX to TX1

// ROS Node Handle
ros::NodeHandle nh;

// ROS Publishers
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

sensor_msgs::NavSatFix navsat_msg;
ros::Publisher navsat_pub("navsatfix", &navsat_msg);

std_msgs::Int32MultiArray enc_msg;
ros::Publisher enc_pub("encoders", &enc_msg);

const int ENCODER_PINS[4][2] = {
  {2, 3},   // Encoder 1: Pins 2 & 3
  {4, 5},   // Encoder 2: Pins 4 & 5
  {16, 17},   // Encoder 3: Pins 6 & 7
  {21, 23}    // Encoder 4: Pins 8 & 9
};

// Array to hold encoder positions for publishing
int32_t encoder_positions[4];

// Structure to hold encoder data
struct EncoderData {
  volatile long position;
  volatile byte previousState;
  unsigned long lastDebounceTime;
};

// Array of encoder data structures
EncoderData encoders[4];

// Debounce delay in milliseconds
const unsigned long debounceDelay = 1;

// Interrupt handlers for each encoder
void encoder1ISR() {
  handleEncoder(0);
}

void encoder2ISR() {
  handleEncoder(1);
}

void encoder3ISR() {
  handleEncoder(2);
}

void encoder4ISR() {
  handleEncoder(3);
}

// Timers
IntervalTimer imuTimer;
IntervalTimer gpsTimer;
IntervalTimer encoderTimer;

void setup() {
    Serial.begin(115200);
    while (!Serial);  // Wait for the serial connection to establish

    // Initialize MPU9250
    Wire.begin();
    if (!mpu.setup(0x68)) {
        Serial.println("MPU connection failed. Check your connection.");
        while (1);
    }

    // Initialize Kalman filters
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

    // Start UART communication with NEO-M8P-2
    gpsSerial.begin(9600);  // Set baud rate to 9600

     enc_msg.data_length = 4;
     enc_msg.data = encoder_positions;

       // Initialize all encoder pins and interrupts
  for (int i = 0; i < 4; i++) {
    // Configure pins as inputs with pullup resistors
    pinMode(ENCODER_PINS[i][0], INPUT_PULLUP);
    pinMode(ENCODER_PINS[i][1], INPUT_PULLUP);
    
    // Initialize encoder data
    encoders[i].position = 0;
    encoders[i].previousState = 0;
    encoders[i].lastDebounceTime = 0;
    encoder_positions[i] = 0;
  }
  
  // Attach interrupts for both pins of each encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0][0]), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0][1]), encoder1ISR, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1][0]), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1][1]), encoder2ISR, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[2][0]), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[2][1]), encoder3ISR, CHANGE);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[3][0]), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[3][1]), encoder4ISR, CHANGE);

    // Initialize ROS node
    nh.initNode();
    nh.advertise(imu_pub);
    nh.advertise(navsat_pub);
     nh.advertise(enc_pub);
    

    // Start timers
    imuTimer.begin(readIMUTask, 15 * 1000);  // IMU update every 25 ms
    gpsTimer.begin(readGPSTask, 25 * 1000); // GPS update every 100 ms
    encoderTImer.begin(readEncoderTask,50*1000)

    Serial.println("Initialization complete.");
}

void loop() {
    nh.spinOnce();  // Process incoming ROS messages
}

// Function to read IMU data and publish it
void readIMUTask() {
    if (mpu.update()) {
        float gyroX = mpu.getGyroX();
        float gyroY = mpu.getGyroY();
        float gyroZ = mpu.getGyroZ();

        // Update Kalman filters
        float dt = 0.025; // Time interval in seconds (25 milliseconds)
        angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
        anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
        angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

        // Convert Euler angles to quaternion
        float q[4];
        float cy = cos(angleYaw * 0.5);
        float sy = sin(angleYaw * 0.5);
        float cp = cos(anglePitch * 0.5);
        float sp = sin(anglePitch * 0.5);
        float cr = cos(angleRoll * 0.5);
        float sr = sin(angleRoll * 0.5);

        q[0] = cr * cp * cy + sr * sp * sy; // q[0] = w
        q[1] = sr * cp * cy - cr * sp * sy; // q[1] = x
        q[2] = cr * sp * cy + sr * cp * sy; // q[2] = y
        q[3] = cr * cp * sy - sr * sp * cy; // q[3] = z

        // Fill the IMU message
        imu_msg.header.stamp = nh.now();
        imu_msg.orientation.x = q[1];
        imu_msg.orientation.y = q[2];
        imu_msg.orientation.z = q[3];
        imu_msg.orientation.w = q[0];
        imu_msg.angular_velocity.x = gyroX;
        imu_msg.angular_velocity.y = gyroY;
        imu_msg.angular_velocity.z = gyroZ;

        // Publish IMU message
        imu_pub.publish(&imu_msg);
    }
}

// Function to read GPS data and publish it
void readGPSTask() {
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

        // Fill status field
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
}

void readEncoderTask(){
    // Publish encoder positions every 50ms
  static unsigned long lastPublishTime = 0;
  if (millis() - lastPublishTime >= 50) {
    // Update array with current positions
    noInterrupts(); // Disable interrupts while copying volatile data
    for(int i = 0; i < 4; i++) {
      encoder_positions[i] = encoders[i].position;
    }
    interrupts(); // Re-enable interrupts
    
    // Publish message
    enc_pub.publish(&enc_msg);
    
    lastPublishTime = millis();
  }
}



// Handler for encoder state changes
void handleEncoder(int encoderIndex) {
  // Debounce
  if (millis() - encoders[encoderIndex].lastDebounceTime < debounceDelay) {
    return;
  }
  encoders[encoderIndex].lastDebounceTime = millis();
  
  // Read current state of encoder pins
  byte currentState = (digitalRead(ENCODER_PINS[encoderIndex][0]) << 1) | 
                      digitalRead(ENCODER_PINS[encoderIndex][1]);
  
  // Determine direction based on state change
  if (encoders[encoderIndex].previousState != currentState) {
    switch (encoders[encoderIndex].previousState) {
      case 0:
        if (currentState == 2) encoders[encoderIndex].position++;
        if (currentState == 1) encoders[encoderIndex].position--;
        break;
      case 1:
        if (currentState == 0) encoders[encoderIndex].position++;
        if (currentState == 3) encoders[encoderIndex].position--;
        break;
      case 2:
        if (currentState == 3) encoders[encoderIndex].position++;
        if (currentState == 0) encoders[encoderIndex].position--;
        break;
      case 3:
        if (currentState == 1) encoders[encoderIndex].position++;
        if (currentState == 2) encoders[encoderIndex].position--;
        break;
    }
    encoders[encoderIndex].previousState = currentState;
  }
}

