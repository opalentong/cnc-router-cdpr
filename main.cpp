#include <Wire.h>
#include <AccelStepper.h>
#include <Encoder.h>

// Define I2C multiplexer address
#define TCA9548A_ADDR 0x70

// Define sensor I2C addresses (assuming default addresses)
#define TOF_SENSOR_X_ADDR 0x52
#define TOF_SENSOR_Y_ADDR 0x53

// Define push button pin
#define BUTTON_PIN PB9

// Dimensions
const int boundarySize = 151;  // cm
const int deviceSize = 30;     // cm

// Variables for storing distances
int distanceX = 0;
int distanceY = 0;

// State of the button
bool buttonState = false;
bool lastButtonState = false;

// Stepper motor pins (adjust as necessary)
#define MOTOR1_STEP_PIN PD0
#define MOTOR1_DIR_PIN PD1
#define MOTOR2_STEP_PIN PD2
#define MOTOR2_DIR_PIN PD3
#define MOTOR3_STEP_PIN PD4
#define MOTOR3_DIR_PIN PD5
#define MOTOR4_STEP_PIN PD6
#define MOTOR4_DIR_PIN PD7
#define MOTOR_Z_STEP_PIN PD8
#define MOTOR_Z_DIR_PIN PD9

// Encoder pins (adjust as necessary)
#define ENCODER1_A_PIN PB0
#define ENCODER1_B_PIN PB1
#define ENCODER2_A_PIN PB3
#define ENCODER2_B_PIN PB4
#define ENCODER3_A_PIN PB5
#define ENCODER3_B_PIN PB6
#define ENCODER4_A_PIN PB7
#define ENCODER4_B_PIN PB8

// MMA7361 pins
#define ACC_X_PIN PA0
#define ACC_Y_PIN PA1
#define ACC_Z_PIN PA2

// Vibration threshold
#define VIBRATION_THRESHOLD 300

// Define the stepper motors
AccelStepper motor1(AccelStepper::DRIVER, MOTOR1_STEP_PIN, MOTOR1_DIR_PIN);
AccelStepper motor2(AccelStepper::DRIVER, MOTOR2_STEP_PIN, MOTOR2_DIR_PIN);
AccelStepper motor3(AccelStepper::DRIVER, MOTOR3_STEP_PIN, MOTOR3_DIR_PIN);
AccelStepper motor4(AccelStepper::DRIVER, MOTOR4_STEP_PIN, MOTOR4_DIR_PIN);
AccelStepper motorZ(AccelStepper::DRIVER, MOTOR_Z_STEP_PIN, MOTOR_Z_DIR_PIN);

// Define the encoders
Encoder encoder1(ENCODER1_A_PIN, ENCODER1_B_PIN);
Encoder encoder2(ENCODER2_A_PIN, ENCODER2_B_PIN);
Encoder encoder3(ENCODER3_A_PIN, ENCODER3_B_PIN);
Encoder encoder4(ENCODER4_A_PIN, ENCODER4_B_PIN);

// Encoder positions
long pos1 = 0;
long pos2 = 0;
long pos3 = 0;
long pos4 = 0;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin();

  // Initialize push button pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Initialize motors
  motor1.setMaxSpeed(1000);
  motor1.setAcceleration(500);
  motor2.setMaxSpeed(1000);
  motor2.setAcceleration(500);
  motor3.setMaxSpeed(1000);
  motor3.setAcceleration(500);
  motor4.setMaxSpeed(1000);
  motor4.setAcceleration(500);
  motorZ.setMaxSpeed(500);
  motorZ.setAcceleration(250);

  // Initialize sensors
  initSensors();
}

void loop() {
  // Read button state
  buttonState = digitalRead(BUTTON_PIN);

  // Check if the button was pressed
  if (buttonState == LOW && lastButtonState == HIGH) {
    // Button was pressed, start autocentering
    autoCenter();
  }

  // Update last button state
  lastButtonState = buttonState;

  // Read distances from sensors
  distanceX = readDistance(0);
  distanceY = readDistance(1);

  // Print distances
  Serial.print("Distance X: ");
  Serial.print(distanceX);
  Serial.print(" cm, Distance Y: ");
  Serial.println(distanceY);

  // Read encoder positions
  pos1 = encoder1.read();
  pos2 = encoder2.read();
  pos3 = encoder3.read();
  pos4 = encoder4.read();

  // Print encoder positions
  Serial.print("Encoder 1: ");
  Serial.print(pos1);
  Serial.print(" Encoder 2: ");
  Serial.print(pos2);
  Serial.print(" Encoder 3: ");
  Serial.print(pos3);
  Serial.print(" Encoder 4: ");
  Serial.println(pos4);

  // Read accelerometer data
  int accX = analogRead(ACC_X_PIN);
  int accY = analogRead(ACC_Y_PIN);
  int accZ = analogRead(ACC_Z_PIN);

  // Calculate total vibration (example calculation)
  int vibration = abs(accX - 512) + abs(accY - 512) + abs(accZ - 512);

  // Check for excessive vibration
  if (vibration > VIBRATION_THRESHOLD) {
    handleExcessiveVibration();
  }

  // Handle serial input for G-code commands
  if (Serial.available() > 0) {
    String gcode = Serial.readStringUntil('\n');
    processGCode(gcode);
  }

  delay(50); // Reduce the delay for faster response
}

void tcaSelect(uint8_t channel) {
  if (channel > 7) return;

  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void initSensors() {
  // Initialize both sensors (X and Y)
  tcaSelect(0);
  initSensor(TOF_SENSOR_X_ADDR);
  tcaSelect(1);
  initSensor(TOF_SENSOR_Y_ADDR);
}

void initSensor(uint8_t address) {
  Wire.beginTransmission(address);
  // Write initialization commands to sensor if needed
  Wire.endTransmission();
}

int readDistance(uint8_t channel) {
  tcaSelect(channel);
  return readSensor(TOF_SENSOR_X_ADDR + channel);  // Adjust address based on channel
}

int readSensor(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00);  // Replace with the appropriate register address if necessary
  Wire.endTransmission();

  Wire.requestFrom(address, 2);
  if (Wire.available() == 2) {
    int distance = Wire.read() << 8;
    distance |= Wire.read();
    return distance;
  }
  return -1;  // Return -1 if failed to read distance
}

void autoCenter() {
  int centerX = (boundarySize - deviceSize) / 2;
  int centerY = (boundarySize - deviceSize) / 2;

  Serial.print("Centering to X: ");
  Serial.print(centerX);
  Serial.print(" cm, Y: ");
  Serial.println(centerY);

  // Calculate motor steps for centering
  long calculateSteps (int centerX, int centerY);
  long stepsX = calculateSteps(centerX, centerY);

  // Move motors to center
  motor1.moveTo(stepsX);
  motor2.moveTo(stepsX);
  motor3.moveTo(stepsX);
  motor4.moveTo(stepsX);

  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0 || motor4.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
  }
}

long calculateSteps(int x, int y) {
  // Convert x, y coordinates to motor steps (implement your conversion logic here)
  long steps = 0;
  // Example: steps = (long)(x * 100); // Adjust conversion factor as necessary
  return steps;
}

void handleExcessiveVibration() {
  Serial.println("Excessive vibration detected! Stopping Z-axis.");

  // Stop the Z-axis motor
  motorZ.stop();

  // Optional: Move Z-axis to a safe position or perform other actions
  motorZ.moveTo(0);  // Example: Move Z-axis to home position
  while (motorZ.distanceToGo() != 0) {
    motorZ.run();
  }
}

void processGCode(String gcode) {
  gcode.trim();
  Serial.println("Received G-code: " + gcode);

  if (gcode.startsWith("G1")) {  // Linear Move
    float x = 0, y = 0, z = 0, f = 0;
    char *str = strdup(gcode.c_str());
    char *token = strtok(str, " ");
    while (token != NULL) {
      if (token[0] == 'X') x = atof(token + 1);
      else if (token[0] == 'Y') y = atof(token + 1);
      else if (token[0] == 'Z') z = atof(token + 1);
      else if (token[0] == 'F') f = atof(token + 1);
      token = strtok(NULL, " ");
    }
    free(str);
    moveTo(x, y, z, f);
  }
  // Add handling for other G-code commands as needed
}

void moveTo(float x, float y, float z, float feedrate) {
  long stepsX = calculateSteps(x, y);  // Example function to calculate steps
  long stepsY = calculateSteps(x, y);  // Adjust this logic as necessary
  long stepsZ = calculateSteps(0, z);  // Example function to calculate Z steps

  motor1.moveTo(stepsX);
  motor2.moveTo(stepsX);
  motor3.moveTo(stepsY);
  motor4.moveTo(stepsY);
  motorZ.moveTo(stepsZ);

  while (motor1.distanceToGo() != 0 || motor2.distanceToGo() != 0 || motor3.distanceToGo() != 0 || motor4.distanceToGo() != 0 || motorZ.distanceToGo() != 0) {
    motor1.run();
    motor2.run();
    motor3.run();
    motor4.run();
    motorZ.run();
  }
}
