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
const float Q_angle = 0.001;
const float Q_bias = 0.003;
const float R_measure = 0.03;

float angleYaw, anglePitch, angleRoll;

// Initialize LCD with I2C address (typically 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Changed to 20x4 LCD for more display space

const int voltagePin = 38;  // Voltage sensor analog input pin
const int currentPin = 41;  // Current sensor analog input pin
const float voltage_reference = 25.6;  // Teensy 4.1 voltage reference is 3.3V
const int adc_resolution = 1023;  // 10-bit ADC resolution

// Current sensor parameters (adjust these based on your sensor specifications)
const float current_sensitivity = 0.185; // V/A for ACS712 30A model (adjust if using different model)
const float current_sensor_offset = voltage_reference / 2; // Offset voltage at 0A (typically Vcc/2)

// GPS Setup
TinyGPSPlus gps;
#define gpsSerial Serial1

// ROS Node Handle
ros::NodeHandle nh;

// ROS Publishers
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu_data", &imu_msg);

sensor_msgs::NavSatFix navsat_msg;
ros::Publisher navsat_pub("navsatfix", &navsat_msg);

std_msgs::Int32MultiArray enc_msg;
ros::Publisher enc_pub("encoders", &enc_msg);

// Encoder Setup
const int ENCODER_PINS[4][2] = {
    {2, 3},   // Encoder 1
    {4, 5},   // Encoder 2
    {16, 17}, // Encoder 3
    {21, 23}  // Encoder 4
};

int32_t encoder_positions[4];

struct EncoderData {
    volatile long position;
    volatile byte previousState;
    unsigned long lastDebounceTime;
};

EncoderData encoders[4];
const unsigned long debounceDelay = 1;

// Motor control pins
const int MOTOR_PINS[4][2] = {
    {8, 9},   // Motor 1: PWM, DIR (Front-Left)
    {6, 7},   // Motor 2: PWM, DIR (Front-Right)
    {15, 14},   // Motor 3: PWM, DIR (Back-Left)
    {22, 20}   // Motor 4: PWM, DIR (Back-Right)
};

// RC channel pins
const int RC_PINS[5] = {
    12, // CH1 - Forward/Backward
    33, // CH2 - Left/Right
    28, // CH3 - Spin Left
    29, // CH4 - Spin Right
    37  // CH5 - Stop
};

// PWM signal range for RC
const int pwmMin = 1000;
const int pwmMax = 2000;
int motorSpeed = 0;

// Timers
IntervalTimer imuTimer;
IntervalTimer gpsTimer;
IntervalTimer encoderTimer;
Intervaltimer lcdTimer;

// Function prototypes
void encoder1ISR() { handleEncoder(0); }
void encoder2ISR() { handleEncoder(1); }
void encoder3ISR() { handleEncoder(2); }
void encoder4ISR() { handleEncoder(3); }

void setup() {
    Serial.begin(115200);
    while (!Serial);

    // Initialize MPU9250
    Wire.begin();
    if (!mpu.setup(0x68)) {
        Serial.println("MPU connection failed");
        while (1);
    }

    // Initialize Kalman filters
    kalmanYaw.setAngle(0);
    kalmanPitch.setAngle(0);
    kalmanRoll.setAngle(0);
    
    for (int i = 0; i < 3; i++) {
        Kalman* filters[] = {&kalmanYaw, &kalmanPitch, &kalmanRoll};
        filters[i]->setQangle(Q_angle);
        filters[i]->setQbias(Q_bias);
        filters[i]->setRmeasure(R_measure);
    }

      // Initialize I2C communication
  Wire.begin();
  Wire.setSDA(25);  // Set SDA pin
  Wire.setSCL(24);  // Set SCL pin
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Set ADC resolution to 10 bits
  analogReadResolution(10);

    // Initialize GPS
    gpsSerial.begin(9600);

    // Initialize encoder message
    enc_msg.data_length = 4;
    enc_msg.data = encoder_positions;

    // Initialize encoders
    for (int i = 0; i < 4; i++) {
        pinMode(ENCODER_PINS[i][0], INPUT_PULLUP);
        pinMode(ENCODER_PINS[i][1], INPUT_PULLUP);
        encoders[i].position = 0;
        encoders[i].previousState = 0;
        encoders[i].lastDebounceTime = 0;
        encoder_positions[i] = 0;
    }

    // Initialize motor pins
    for (int i = 0; i < 4; i++) {
        pinMode(MOTOR_PINS[i][0], OUTPUT); // PWM
        pinMode(MOTOR_PINS[i][1], OUTPUT); // DIR
        analogWrite(MOTOR_PINS[i][0], 0);  // Start with motors stopped
    }

    // Initialize RC pins
    for (int i = 0; i < 5; i++) {
        pinMode(RC_PINS[i], INPUT);
    }

    // Attach encoder interrupts
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

    // Start sensor timers
    imuTimer.begin(readIMUTask, 15000);    // 15ms
    gpsTimer.begin(readGPSTask, 25000);    // 25ms
    encoderTimer.begin(readEncoderTask, 5000); // 50ms
    lcdTimer.begin(lcd_display,2000);

    Serial.println("Initialization complete.");
}

void loop() {
    // Process ROS messages
    nh.spinOnce();

    // Read RC channels
    int ch1 = pulseIn(RC_PINS[0], HIGH);  // Forward/Backward
    int ch2 = pulseIn(RC_PINS[1], HIGH);  // Left/Right
    int ch3 = pulseIn(RC_PINS[2], HIGH);  // Spin Left
    int ch4 = pulseIn(RC_PINS[3], HIGH);  // Spin Right
    int ch5 = pulseIn(RC_PINS[4], HIGH);  // Emergency Stop

    // Emergency stop check
    if (ch5 > 1500) {
        stopMotors();
        return;
    }

    // Calculate motor speed based on channel 1 (throttle)
    if (ch1 > 1500) {
        motorSpeed = map(ch1, 1500, pwmMax, 0, 255);
    } else if (ch1 < 1500) {
        motorSpeed = map(ch1, pwmMin, 1500, 255, 0);
    } else {
        motorSpeed = 0;
    }

    // Movement control based on RC inputs
    if (ch1 > 1600) {         // Forward (with deadzone)
        moveForward();
    } 
    else if (ch1 < 1400) {    // Backward (with deadzone)
        moveBackward();
    }
    else if (ch2 > 1600) {    // Right (with deadzone)
        moveRight();
    }
    else if (ch2 < 1400) {    // Left (with deadzone)
        moveLeft();
    }
    else if (ch3 > 1600) {    // Spin Left
        spinLeft();
    }
    else if (ch4 > 1600) {    // Spin Right
        spinRight();
    }
    else {
        stopMotors();
    }

    // Small delay to avoid overwhelming the system
    delay(10);
}

void readIMUTask() {
if (mpu.update()) {
    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    float gyroZ = mpu.getGyroZ();

    float dt = 0.015; // 15ms interval
    angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
    anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
    angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

    // Print angles for debugging
    Serial.print("Yaw: "); Serial.print(angleYaw);
    Serial.print(" Pitch: "); Serial.print(anglePitch);
    Serial.print(" Roll: "); Serial.println(angleRoll);

    // Convert to quaternion
    float q[4];
    float cy = cos(angleYaw * 0.5);
    float sy = sin(angleYaw * 0.5);
    float cp = cos(anglePitch * 0.5);
    float sp = sin(anglePitch * 0.5);
    float cr = cos(angleRoll * 0.5);
    float sr = sin(angleRoll * 0.5);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;

    // Print quaternion values for debugging
    Serial.print("Quaternion - w: "); Serial.print(q[0]);
    Serial.print(" x: "); Serial.print(q[1]);
    Serial.print(" y: "); Serial.print(q[2]);
    Serial.print(" z: "); Serial.println(q[3]);

    // Publish data to ROS
    imu_msg.header.stamp = nh.now();
    imu_msg.orientation.x = q[1];
    imu_msg.orientation.y = q[2];
    imu_msg.orientation.z = q[3];
    imu_msg.orientation.w = q[0];
    imu_msg.angular_velocity.x = gyroX;
    imu_msg.angular_velocity.y = gyroY;
    imu_msg.angular_velocity.z = gyroZ;

    imu_pub.publish(&imu_msg);

    // Print angular velocity values for debugging
    Serial.print("Angular Velocity - X: "); Serial.print(gyroX);
    Serial.print(" Y: "); Serial.print(gyroY);
    Serial.print(" Z: "); Serial.println(gyroZ);
}

}

void readGPSTask() {
 while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
}

if (gps.location.isUpdated()) {
    navsat_msg.latitude = gps.location.lat();
    navsat_msg.longitude = gps.location.lng();
    navsat_msg.altitude = gps.altitude.meters();
    navsat_msg.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
    navsat_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    navsat_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
    
    // Serial printing GPS data
    Serial.print("Latitude: ");
    Serial.println(navsat_msg.latitude);
    Serial.print("Longitude: ");
    Serial.println(navsat_msg.longitude);
    Serial.print("Altitude: ");
    Serial.println(navsat_msg.altitude);

    navsat_pub.publish(&navsat_msg);
}

}


void lcd_display(){
  // Read voltage sensor
  int voltageRaw = analogRead(voltagePin);
  float voltage = (voltageRaw * voltage_reference) / adc_resolution;
  
  // Read current sensor
  int currentRaw = analogRead(currentPin);
  float sensorVoltage = (currentRaw * voltage_reference) / adc_resolution;
  
  // Convert sensor voltage to current
  // For ACS712: I = (V - Voffset) / Sensitivity
  float current = (sensorVoltage - current_sensor_offset) / current_sensitivity;
  
  // Clear LCD and display values
  lcd.clear();
  
  // Display voltage on first line
  lcd.setCursor(0, 0);
  lcd.print("Voltage: ");
  lcd.print(voltage, 2);  // Display with 2 decimal places
  lcd.print("V");
  
  // Display current on second line
  lcd.setCursor(0, 1);
  lcd.print("Current: ");
  lcd.print(abs(current), 2);  // Using abs() to show magnitude
  lcd.print("A");
  
  // Display power on third line
  float power = voltage * abs(current);
  lcd.setCursor(0, 2);
  lcd.print("Power: ");
  lcd.print(power, 2);
  lcd.print("W");
}

void readEncoderTask() {
    noInterrupts();
for(int i = 0; i < 4; i++) {
    encoder_positions[i] = encoders[i].position;
    Serial.print("Encoder ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(encoder_positions[i]);
}
interrupts();

enc_pub.publish(&enc_msg);
}

void handleEncoder(int encoderIndex) {
    if (millis() - encoders[encoderIndex].lastDebounceTime < debounceDelay) {
        return;
    }
    encoders[encoderIndex].lastDebounceTime = millis();
    
    byte currentState = (digitalRead(ENCODER_PINS[encoderIndex][0]) << 1) | 
                        digitalRead(ENCODER_PINS[encoderIndex][1]);
    
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

// Motor control functions
void moveForward() {
    for (int i = 0; i < 4; i++) {
        motorForward(MOTOR_PINS[i][0], MOTOR_PINS[i][1]);
    }
}

void moveBackward() {
    for (int i = 0; i < 4; i++) {
        motorBackward(MOTOR_PINS[i][0], MOTOR_PINS[i][1]);
    }
}

void moveLeft() {
    motorBackward(MOTOR_PINS[0][0], MOTOR_PINS[0][1]); // Front-Left
    motorForward(MOTOR_PINS[1][0], MOTOR_PINS[1][1]);  // Front-Right
    motorForward(MOTOR_PINS[2][0], MOTOR_PINS[2][1]);  // Back-Left
    motorBackward(MOTOR_PINS[3][0], MOTOR_PINS[3][1]); // Back-Right
}

void moveRight() {
    motorForward(MOTOR_PINS[0][0], MOTOR_PINS[0][1]);  // Front-Left
    motorBackward(MOTOR_PINS[1][0], MOTOR_PINS[1][1]); // Front-Right
    motorBackward(MOTOR_PINS[2][0], MOTOR_PINS[2][1]); // Back-Left
    motorForward(MOTOR_PINS[3][0], MOTOR_PINS[3][1]);  // Back-Right
}

void spinLeft() {
    motorBackward(MOTOR_PINS[0][0], MOTOR_PINS[0][1]); // Front-Left
    motorForward(MOTOR_PINS[1][0], MOTOR_PINS[1][1]);  // Front-Right
    motorBackward(MOTOR_PINS[2][0], MOTOR_PINS[2][1]); // Back-Left
    motorForward(MOTOR_PINS[3][0], MOTOR_PINS[3][1]);  // Back-Right
}

void spinRight() {
    motorForward(MOTOR_PINS[0][0], MOTOR_PINS[0][1]);  // Front-Left
    motorBackward(MOTOR_PINS[1][0], MOTOR_PINS[1][1]); // Front-Right
    motorForward(MOTOR_PINS[2][0], MOTOR_PINS[2][1]);  // Back-Left
    motorBackward(MOTOR_PINS[3][0], MOTOR_PINS[3][1]); // Back-Right
}

void stopMotors() {
    for (int i = 0; i < 4; i++) {
        analogWrite(MOTOR_PINS[i][0], 0);
    }
}

void motorForward(int pwmPin, int dirPin) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, motorSpeed);
}

void motorBackward(int pwmPin, int dirPin) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, motorSpeed);
}
