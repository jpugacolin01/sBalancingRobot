#include <Arduino.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// ===== LCD CONFIGURATION =====
const int LCD_RS = 3;
const int LCD_EN = 4;
const int LCD_D4 = 5;
const int LCD_D5 = 6;
const int LCD_D6 = 7;
const int LCD_D7 = 8;
const int LCD_COLS = 16;
const int LCD_ROWS = 2;
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ===== MOTOR DRIVER PINS (L298N) =====
const int ENA = 44;  // Motor A Enable
const int IN1 = 23;  // Motor A Direction 1
const int IN2 = 25;  // Motor A Direction 2
const int ENB = 46;  // Motor B Enable
const int IN3 = 27;  // Motor B Direction 1
const int IN4 = 29;  // Motor B Direction 2

// ===== MOTOR CONSTANTS =====
const int MIN_MOTOR_SPEED = 55;
const int MAX_MOTOR_SPEED = 150;
const float MOTOR_B_COMPENSATION = 0.925f;  // Right motor runs faster, so reduce speed

// ===== SENSOR SETUP =====
MPU6050 mpu6050(Wire);
float gyroOffsetY = 0.0f;

// ===== PD CONTROLLER PARAMETERS =====
struct PDController {
  float setpoint = 0.0f;
  float kp = 1.25f;
  float kd = 0.025f;
  float lastError = 0.0f;
  float output = 55.0f;
  float acceptableRange = 0.0f;
};

// ===== TIMING CONTROL =====
const unsigned long CONTROL_INTERVAL_MS = 25;
unsigned long lastControlTime = 0;

// ===== ROBOT STATE =====
enum MotorDirection {
  FORWARD = 0,
  BACKWARD = 1,
  STOPPED = 2
};

// Global instances
PDController controller;
MotorDirection currentDirection = STOPPED;

// ===== FUNCTION DECLARATIONS =====
void initializeHardware();
void calibrateSensor();
void startupSequence();
void runBalanceControl();
void updatePDController(float currentAngle);
void setMotorDirection(MotorDirection direction);
void setMotorSpeeds(int speedA, int speedB);
void stopMotors();
void updateDisplay(float angle, float error, float output);
void displayFloat(int col, int row, const char* label, float value);

void setup() {
  Serial.begin(115200);
  initializeHardware();
  calibrateSensor();
  startupSequence();
}

void loop() {
  runBalanceControl();
}

void initializeHardware() {
  // Initialize LCD
  lcd.begin(LCD_COLS, LCD_ROWS);
  
  // Initialize motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize motors to OFF state
  stopMotors();
  
  // Initialize IMU
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(false);
  
  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}

void calibrateSensor() {
  gyroOffsetY = mpu6050.getAngleY();
}

void startupSequence() {
  for (int countdown = 3; countdown > 0; countdown--) {
    Serial.print("Starting in: ");
    Serial.println(countdown);
    
    // Blink LED during countdown
    for (int blinks = 0; blinks < countdown * 2; blinks++) {
      digitalWrite(LED_BUILTIN, blinks % 2);
      delay(250);
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void runBalanceControl() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastControlTime >= CONTROL_INTERVAL_MS) {
    mpu6050.update();
    float currentAngle = mpu6050.getAngleY() - gyroOffsetY;
    
    updatePDController(currentAngle);
    updateDisplay(currentAngle, controller.setpoint - currentAngle, controller.output);
    
    lastControlTime = currentTime;
  }
}

void updatePDController(float currentAngle) {
  // Calculate error
  float error = controller.setpoint - currentAngle;
  
  // Calculate PD terms
  float proportional = controller.kp * error;
  float derivative = (controller.kd / (CONTROL_INTERVAL_MS / 1000.0f)) * (error - controller.lastError);
  
  // Update output
  controller.output += proportional + derivative;
  controller.output = constrain(controller.output, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  
  // Determine motor direction and apply control
  if (currentAngle > controller.acceptableRange) {
    // Leaning backward - move backward
    setMotorDirection(BACKWARD);
    setMotorSpeeds(abs(controller.output), abs(controller.output * MOTOR_B_COMPENSATION));
  } 
  else if (currentAngle < -controller.acceptableRange) {
    // Leaning forward - move forward
    setMotorDirection(FORWARD);
    setMotorSpeeds(abs(controller.output), abs(controller.output * MOTOR_B_COMPENSATION));
  } 
  else {
    // Within acceptable range - stop
    stopMotors();
  }
  
  controller.lastError = error;
}

void setMotorDirection(MotorDirection direction) {
  currentDirection = direction;
  
  switch (direction) {
    case FORWARD:
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      break;
      
    case BACKWARD:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      break;
      
    case STOPPED:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
  }
}

void setMotorSpeeds(int speedA, int speedB) {
  analogWrite(ENA, speedA);
  analogWrite(ENB, speedB);
}

void stopMotors() {
  setMotorDirection(STOPPED);
  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);
}

void updateDisplay(float angle, float error, float output) {
  lcd.clear();
  
  // First row: Process Value and Error
  displayFloat(0, 0, "pv=", angle);
  displayFloat(8, 0, "err=", error);
  
  // Second row: Setpoint and Output
  displayFloat(0, 1, "sp=", controller.setpoint);
  displayFloat(8, 1, "out=", output);
}

void displayFloat(int col, int row, const char* label, float value) {
  const int DECIMAL_PLACES = 100;
  int truncated = value * DECIMAL_PLACES;
  
  lcd.setCursor(col, row);
  lcd.print(label);
  lcd.print(truncated / DECIMAL_PLACES);
  lcd.print(".");
  lcd.print(abs(truncated) % DECIMAL_PLACES);
}
