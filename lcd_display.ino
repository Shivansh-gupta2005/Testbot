#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD with I2C address (typically 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 20, 4);  // Changed to 20x4 LCD for more display space

const int voltagePin = 38;  // Voltage sensor analog input pin
const int currentPin = 41;  // Current sensor analog input pin
const float voltage_reference = 25.6;  // Teensy 4.1 voltage reference is 3.3V
const int adc_resolution = 1023;  // 10-bit ADC resolution

// Current sensor parameters (adjust these based on your sensor specifications)
const float current_sensitivity = 0.185; // V/A for ACS712 30A model (adjust if using different model)
const float current_sensor_offset = voltage_reference / 2; // Offset voltage at 0A (typically Vcc/2)

void setup() {
  // Initialize I2C communication
  Wire.begin();
  Wire.setSDA(24);  // Set SDA pin
  Wire.setSCL(25);  // Set SCL pin
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  
  // Set ADC resolution to 10 bits
  analogReadResolution(10);
  
  // Optional: Initialize Serial for debugging
  Serial.begin(9600);
}

void loop() {
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
  
  // Optional: Print to Serial for debugging
  Serial.print("Voltage: ");
  Serial.print(voltage);
  Serial.print("V, Current: ");
  Serial.print(current);
  Serial.print("A, Power: ");
  Serial.print(power);
  Serial.println("W");
  
  delay(500);  // Update every 500ms
}
