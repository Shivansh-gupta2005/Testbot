#include <ros.h>
#include <std_msgs/Int32.h>

// ROS node handle
ros::NodeHandle nh;

// ROS messages for each encoder
std_msgs::Int32 enc1_msg;
std_msgs::Int32 enc2_msg;
std_msgs::Int32 enc3_msg;
std_msgs::Int32 enc4_msg;

// ROS publishers for each encoder
ros::Publisher enc1_pub("encoder1", &enc1_msg);
ros::Publisher enc2_pub("encoder2", &enc2_msg);
ros::Publisher enc3_pub("encoder3", &enc3_msg);
ros::Publisher enc4_pub("encoder4", &enc4_msg);

// Define encoder pins for all 4 encoders
const int ENCODER_PINS[4][2] = {
  {2, 3},   // Encoder 1: Pins 2 & 3
  {4, 5},   // Encoder 2: Pins 4 & 5
  {16, 17},   // Encoder 3: Pins 6 & 7
  {21, 23}    // Encoder 4: Pins 8 & 9
};

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

void setup() {
  // Initialize ROS node
  nh.initNode();
  
  // Advertise publishers
  nh.advertise(enc1_pub);
  nh.advertise(enc2_pub);
  nh.advertise(enc3_pub);
  nh.advertise(enc4_pub);
  
  // Initialize all encoder pins and interrupts
  for (int i = 0; i < 4; i++) {
    // Configure pins as inputs with pullup resistors
    pinMode(ENCODER_PINS[i][0], INPUT_PULLUP);
    pinMode(ENCODER_PINS[i][1], INPUT_PULLUP);
    
    // Initialize encoder data
    encoders[i].position = 0;
    encoders[i].previousState = 0;
    encoders[i].lastDebounceTime = 0;
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
}

void loop() {
  // Publish encoder positions every 50ms
  static unsigned long lastPublishTime = 0;
  if (millis() - lastPublishTime >= 50) {
    // Update messages with current positions
    enc1_msg.data = encoders[0].position;
    enc2_msg.data = encoders[1].position;
    enc3_msg.data = encoders[2].position;
    enc4_msg.data = encoders[3].position;
    
    // Publish all messages
    enc1_pub.publish(&enc1_msg);
    enc2_pub.publish(&enc2_msg);
    enc3_pub.publish(&enc3_msg);
    enc4_pub.publish(&enc4_msg);
    
    lastPublishTime = millis();
  }
  
  // Process ROS callbacks
  nh.spinOnce();
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

// Optional: Function to reset specific encoder position
void resetEncoder(int encoderIndex) {
  if (encoderIndex >= 0 && encoderIndex < 4) {
    encoders[encoderIndex].position = 0;
  }
}

// Optional: Function to reset all encoder positions
void resetAllEncoders() {
  for (int i = 0; i < 4; i++) {
    encoders[i].position = 0;
  }
}
