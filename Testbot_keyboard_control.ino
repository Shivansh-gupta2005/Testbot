const int MOTOR_PINS[4][2] = {
    {8, 9},
    {6, 7},
    {15, 14},
    {22, 20}
};

const int ENCODER_PINS[4][2] = {
    {2, 3},
    {4, 5},
    {16, 17},
    {21, 23}
};

volatile long encoder_positions[4] = {0, 0, 0, 0};
const int motorSpeed = 1023;

void setup() {
    Serial.begin(115200);
    
    for (int i = 0; i < 4; i++) {
        pinMode(MOTOR_PINS[i][0], OUTPUT);
        pinMode(MOTOR_PINS[i][1], OUTPUT);
        analogWrite(MOTOR_PINS[i][0], 0);
    }
    
    for (int i = 0; i < 4; i++) {
        pinMode(ENCODER_PINS[i][0], INPUT_PULLUP);
        pinMode(ENCODER_PINS[i][1], INPUT_PULLUP);
    }
    
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0][0]), encoder1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0][1]), encoder1_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1][0]), encoder2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1][1]), encoder2_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[2][0]), encoder3_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[2][1]), encoder3_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[3][0]), encoder4_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[3][1]), encoder4_ISR, CHANGE);
}

void loop() {
    if (Serial.available()) {
        char cmd = Serial.read();
        switch (cmd) {
            case 'w':
                moveForward();
                break;
            case 's':
                moveBackward();
                break;
            case 'a':
                moveLeft();
                break;
            case 'd':
                moveRight();
                break;
            case 'q':
                spinLeft();
                break;
            case 'e':
                spinRight();
                break;
            case ' ':
                stopMotors();
                break;
        }
    }
    
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
        printEncoderValues();
        lastPrint = millis();
    }
}

void encoder1_ISR() {
    updateEncoder(0);
}

void encoder2_ISR() {
    updateEncoder(1);
}

void encoder3_ISR() {
    updateEncoder(2);
}

void encoder4_ISR() {
    updateEncoder(3);
}

void updateEncoder(int index) {
    bool A = digitalRead(ENCODER_PINS[index][0]);
    bool B = digitalRead(ENCODER_PINS[index][1]);
    
    if (A == B) {
        encoder_positions[index]++;
    } else {
        encoder_positions[index]--;
    }
}

void printEncoderValues() {
    for (int i = 0; i < 4; i++) {
        Serial.print("Encoder ");
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(encoder_positions[i]);
        Serial.print("\t");
    }
    Serial.println();
}

void moveForward() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(MOTOR_PINS[i][1], HIGH);
        analogWrite(MOTOR_PINS[i][0], motorSpeed);
    }
}

void moveBackward() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(MOTOR_PINS[i][1], LOW);
        analogWrite(MOTOR_PINS[i][0], motorSpeed);
    }
}

void moveLeft() {
    digitalWrite(MOTOR_PINS[0][1], LOW);
    digitalWrite(MOTOR_PINS[2][1], LOW);
    digitalWrite(MOTOR_PINS[1][1], HIGH);
    digitalWrite(MOTOR_PINS[3][1], HIGH);
    
    for (int i = 0; i < 4; i++) {
        analogWrite(MOTOR_PINS[i][0], motorSpeed);
    }
}

void moveRight() {
    digitalWrite(MOTOR_PINS[0][1], HIGH);
    digitalWrite(MOTOR_PINS[2][1], HIGH);
    digitalWrite(MOTOR_PINS[1][1], LOW);
    digitalWrite(MOTOR_PINS[3][1], LOW);
    
    for (int i = 0; i < 4; i++) {
        analogWrite(MOTOR_PINS[i][0], motorSpeed);
    }
}

void spinLeft() {
    digitalWrite(MOTOR_PINS[0][1], LOW);
    digitalWrite(MOTOR_PINS[2][1], LOW);
    digitalWrite(MOTOR_PINS[1][1], HIGH);
    digitalWrite(MOTOR_PINS[3][1], HIGH);
    
    for (int i = 0; i < 4; i++) {
        analogWrite(MOTOR_PINS[i][0], motorSpeed);
    }
}

void spinRight() {
    digitalWrite(MOTOR_PINS[0][1], HIGH);
    digitalWrite(MOTOR_PINS[2][1], HIGH);
    digitalWrite(MOTOR_PINS[1][1], LOW);
    digitalWrite(MOTOR_PINS[3][1], LOW);
    
    for (int i = 0; i < 4; i++) {
        analogWrite(MOTOR_PINS[i][0], motorSpeed);
    }
}

void stopMotors() {
    for (int i = 0; i < 4; i++) {
        analogWrite(MOTOR_PINS[i][0], 0);
    }
}