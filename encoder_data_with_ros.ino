#include <ros.h>
#include <std_msgs/Int32MultiArray.h>

// ROS node handle
ros::NodeHandle nh;

// ROS message for all encoders
std_msgs::Int32MultiArray enc_msg;

// ROS publisher for encoder data
ros::Publisher enc_pub("encoders", &enc_msg);

// Define encoder pins for all 4 encoders
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

void setup() {
  // Initialize ROS node
  nh.initNode();
  
  // Initialize the message array
  enc_msg.data_length = 4;
  enc_msg.data = encoder_positions;
  
  // Advertise publisher
  nh.advertise(enc_pub);
  
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
}

void loop() {
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
    encoder_positions[encoderIndex] = 0;
  }
}

// Optional: Function to reset all encoder positions
void resetAllEncoders() {
  for (int i = 0; i < 4; i++) {
    encoders[i].position = 0;
    encoder_positions[i] = 0;
  }
}
