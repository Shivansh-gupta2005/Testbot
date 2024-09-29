#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <IntervalTimer.h>
#include <util/atomic.h> // For ATOMIC_BLOCK macro
#include <MPU9250.h>
#include <Kalman.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// Initialize the LCD. Adjust the address (usually 0x27 or 0x3F for I2C displays)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Constants for voltage measurement (adjust based on your voltage divider setup)
const int voltagePin = A0;  // Analog pin for voltage sensor
const float vRef = 3.3;     // Teensy reference voltage
const float voltageDividerRatio = 5.0; // Ratio for the voltage divider

// Constants for current measurement (e.g., ACS712)
const int currentPin = A1;   // Analog pin for current sensor
const float mVperAmp = 185.0;  // Sensitivity of current sensor (185mV per amp for ACS712-5A)

// Encoder Connections
#define ENCA1 4 
#define ENCB1 5
#define ENCA2 10
#define ENCB2 11
#define ENCA3 35
#define ENCB3 34
#define ENCA4 36
#define ENCB4 37

// Motor Driver Connections
#define DIR1 2
#define PWM1 3
#define DIR2 6
#define PWM2 9
#define DIR3 20
#define PWM3 22
#define DIR4 21
#define PWM4 23

volatile int posi1 = 0; 
volatile int posi2 = 0;
volatile int posi3 = 0;
volatile int posi4 = 0;

// MPU9250 and Kalman filter objects
MPU9250 mpu;
Kalman kalmanYaw, kalmanPitch, kalmanRoll;

// Kalman filter parameters
float Q_angle = 0.001;   // Process noise covariance for angle
float Q_bias = 0.003;    // Process noise covariance for bias
float R_measure = 0.03;  // Measurement noise covariance

// State variables for yaw, pitch, roll
float angleYaw, anglePitch, angleRoll;
float gyroBiasYaw = 0, gyroBiasPitch = 0, gyroBiasRoll = 0;

// GPS Connections
const int RX_PIN = 28;  // Connect to TX of GPS module
const int TX_PIN = 29;  // Connect to RX of GPS module

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
TinyGPSPlus gps;

// Interval timers
IntervalTimer timer1, timer2, timer3, timer4;

// Function prototypes
void task1();
void task2();
void task3();
void task4();

void setup() {
  // Start the serial monitor for debugging
  Serial.begin(9600);
  
  // Initialize the LCD
  lcd.begin();
  lcd.backlight(); // Turn on the backlight for the LCD

  // Print initial message on LCD
  lcd.setCursor(0, 0);
  lcd.print("Voltage:");
  lcd.setCursor(0, 1);
  lcd.print("Current:");

  // Set up each timer with the respective intervals (in microseconds)
  timer1.begin(task1, 1000000); // Task 1: Every 1 second (1,000,000 µs)
  timer2.begin(task2, 500000);  // Task 2: Every 0.5 seconds (500,000 µs)
  timer3.begin(task3, 25000);   // Task 3: Every 25 milliseconds (25,000 µs)
  timer4.begin(task4, 4000000); // Task 4: Every 4 seconds (4,000,000 µs)

  // Setup Encoder Pins
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(ENCA3, INPUT);
  pinMode(ENCB3, INPUT);
  pinMode(ENCA4, INPUT);
  pinMode(ENCB4, INPUT);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA3), readEncoder3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCA4), readEncoder4, RISING);

  // Setup Motor Driver Pins
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR4, OUTPUT);

  digitalWrite(DIR1,LOW);
  digitalWrite(PWM1,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(PWM2,LOW);
  digitalWrite(DIR3,LOW);
  digitalWrite(PWM3,LOW);
  digitalWrite(DIR4,LOW);
  digitalWrite(PWM4,LOW);

  // MPU9250 initialization
  Wire.begin();
  if (!mpu.setup(0x68)) {
    while (1) {
      Serial.println("MPU connection failed. Please check your connection.");
      delay(5000);
    }
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

  // Initialize GPS module
  gpsSerial.begin(9600);  // Serial communication with the GPS module
}

void loop() {
  moveforward();
  delay(8000);
  stopmotors();
  delay(1500);
  movebackward();
  delay(8000);
  stopmotors();
  delay(1500);
  moveleft();
  delay(8000);
  stopmotors();
  delay(1500);
  moveright();
  delay(8000);
  stopmotors();
  delay(1500);
   spinleft();
  delay(8000);
  stopmotors();
  delay(1500);
  spinright();
  delay(8000);
  stopmotors();
  delay(1500);
  // Main loop remains empty as tasks are handled by interval timers
}

// Task 1: Voltage and current monitoring every 1 second
void task1() {
  // Voltage reading
  int voltageRaw = analogRead(voltagePin);
  float voltage = (voltageRaw / 1023.0) * vRef * voltageDividerRatio;
  
  // Current reading
  int currentRaw = analogRead(currentPin);
  float current = ((currentRaw / 1023.0) * vRef - 2.5) / (mVperAmp / 1000.0);

  // Print the voltage and current to the LCD
  lcd.setCursor(9, 0);  // Move to appropriate position for voltage
  lcd.print(voltage, 2); // Print voltage with 2 decimal places
  lcd.print(" V");

  lcd.setCursor(9, 1);  // Move to appropriate position for current
  lcd.print(current, 2); // Print current with 2 decimal places
  lcd.print(" A");

  // Print the voltage and current to the serial monitor for debugging
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print(" V, Current: ");
  Serial.print(current);
  Serial.println(" A");
}

// Task 2: Encoder readings every 0.5 seconds
void task2() {
  int pos1, pos2, pos3, pos4;
  
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = posi1;
    pos2 = posi2;
    pos3 = posi3;
    pos4 = posi4;
  }
  
  // Print encoder positions to the serial monitor
  Serial.print("Encoder 1: ");
  Serial.print(pos1);
  Serial.print(", Encoder 2: ");
  Serial.print(pos2);
  Serial.print(", Encoder 3: ");
  Serial.print(pos3);
  Serial.print(", Encoder 4: ");
  Serial.println(pos4);
}

// Task 3: MPU9250 readings with Kalman filter every 25 ms
void task3() {
  if (mpu.update()) {
    float gyroX = mpu.getGyroX();
    float gyroY = mpu.getGyroY();
    float gyroZ = mpu.getGyroZ();

    // Update Kalman filters
    float dt = 0.025; // Time interval in seconds (25 milliseconds)
    angleYaw = kalmanYaw.getAngle(mpu.getYaw(), gyroX, dt);
    anglePitch = kalmanPitch.getAngle(mpu.getPitch(), gyroY, dt);
    angleRoll = kalmanRoll.getAngle(mpu.getRoll(), gyroZ, dt);

    // Print filtered angles
    Serial.print("Filtered Yaw, Pitch, Roll: ");
    Serial.print(angleYaw, 2);
    Serial.print(", ");
    Serial.print(anglePitch, 2);
    Serial.print(", ");
    Serial.println(angleRoll, 2);
  }
}

// Task 4: GPS data reading every 4 seconds
void task4() {
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
    delay(1000);  // Delay for readability and avoiding spamming the error message
  }
}

// Interrupt Service Routines for encoders
void readEncoder1() {
  int b = digitalRead(ENCB1);
  if (b > 0) {
    posi1++;
  } else {
    posi1--;
  }
}

void readEncoder2() {
  int b = digitalRead(ENCB2);
  if (b > 0) {
    posi2++;
  } else {
    posi2--;
  }
}

void readEncoder3() {
  int b = digitalRead(ENCB3);
  if (b > 0) {
    posi3++;
  } else {
    posi3--;
  }
}

void readEncoder4() {
  int b = digitalRead(ENCB4);
  if (b > 0) {
    posi4++;
  } else {
    posi4--;
  }
}

void moveforward(){
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,  LOW);
  digitalWrite(DIR4,LOW);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,0);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,0);
}

void moveleft(){ 
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,HIGH);
  digitalWrite(DIR3,LOW);
  digitalWrite(DIR4,HIGH);
  analogWrite(PWM1,0);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,0);
  analogWrite(PWM4,1024);
}

void movebackward(){
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,HIGH);
  digitalWrite(DIR4,LOW);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,0);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,0);
}

void moveright(){
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,LOW);
  digitalWrite(DIR4,LOW);
  analogWrite(PWM1,0);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,0);
  analogWrite(PWM4,1024);
}

void spinleft(){
  digitalWrite(DIR1,LOW);
  digitalWrite(DIR2,HIGH);
  digitalWrite(DIR3,LOW);
  digitalWrite(DIR4,LOW);digitalWrite(PWM1,HIGH);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,1024);
  
}

 void spinright(){
  digitalWrite(DIR1,HIGH);
  digitalWrite(DIR2,LOW);
  digitalWrite(DIR3,HIGH);
  digitalWrite(DIR4,HIGH);
  analogWrite(PWM1,1024);
  analogWrite(PWM2,1024);
  analogWrite(PWM3,1024);
  analogWrite(PWM4,1024);
}


void stopmotors(){

  analogWrite(PWM1,0);
  analogWrite(PWM2,0);
  analogWrite(PWM3,0);
  analogWrite(PWM4,0);

}

