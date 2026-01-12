#include <Arduino.h>
#include <Wire.h>
#include <RF24.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <math.h>

// Pin Definitions
#define MOTOR_FL 6
#define MOTOR_FR 9
#define MOTOR_BL 5
#define MOTOR_BR 3
#define BUZZER_PIN 8

// NRF24L01 Pins
#define CE_PIN 7
#define CSN_PIN 10

// MPU6050
MPU6050 mpu;

// NRF24L01
RF24 radio(CE_PIN, CSN_PIN);

// Radio Address
const byte address[6] = "00001";

// PID Variables
float roll, pitch, yaw;
float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;

// PID Constants
float Kp = 1.0, Ki = 0.05, Kd = 0.1;
float rollError, pitchError, yawError;
float rollPrevError = 0, pitchPrevError = 0, yawPrevError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;

// Motor speeds (0-255)
int motorFL = 0, motorFR = 0, motorBL = 0, motorBR = 0;
int baseThrottle = 0;

// Control structure for radio communication
struct ControlData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte buttons;
};

ControlData rxData;

// Timing variables
unsigned long prevTime = 0;
float dt = 0.0;

// Calibration offsets
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

// Complementary filter variables
float angleX = 0, angleY = 0, angleZ = 0;
float compAngleX = 0, compAngleY = 0;
float alpha = 0.96; // Complementary filter constant

void setup() {
  // Initialize Serial
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_BL, OUTPUT);
  pinMode(MOTOR_BR, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  
  // Initialize motors to 0
  analogWrite(MOTOR_FL, 0);
  analogWrite(MOTOR_FR, 0);
  analogWrite(MOTOR_BL, 0);
  analogWrite(MOTOR_BR, 0);
  
  // Buzzer startup beep
  buzzerBeep(3);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
    buzzerBeep(2);
  } else {
    Serial.println("MPU6050 connection failed");
    while(1) {
      buzzerBeep(1);
      delay(500);
    }
  }
  
  // Calibrate MPU6050
  calibrateMPU();
  
  // Initialize NRF24L01
  Serial.println("Initializing NRF24L01...");
  if (radio.begin()) {
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_LOW); // Use RF24_PA_HIGH for longer range
    radio.setDataRate(RF24_250KBPS);
    radio.startListening();
    Serial.println("NRF24L01 initialized successfully");
    buzzerBeep(4);
  } else {
    Serial.println("NRF24L01 initialization failed");
    while(1) {
      buzzerBeep(1);
      delay(1000);
    }
  }
  
  delay(2000); // Allow everything to stabilize
}

void loop() {
  unsigned long currentTime = micros();
  dt = (currentTime - prevTime) / 1000000.0;
  prevTime = currentTime;
  
  // Read radio data
  readRadioData();
  
  // Read MPU6050 data
  readMPUData();
  
  // Calculate PID
  calculatePID();
  
  // Mix motors
  mixMotors();
  
  // Write motor speeds
  writeMotors();
  
  // Safety checks
  safetyCheck();
}

void readRadioData() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    
    // Map receiver values to appropriate ranges
    baseThrottle = map(rxData.throttle, 0, 255, 1000, 2000); // Convert to microseconds
    rollSetpoint = map(rxData.roll, 0, 255, -20.0, 20.0);    // ±20 degrees
    pitchSetpoint = map(rxData.pitch, 0, 255, -20.0, 20.0);  // ±20 degrees
    yawSetpoint = map(rxData.yaw, 0, 255, -100.0, 100.0);    // Yaw rate
    
    // Arm/disarm with button (using first bit of buttons byte)
    if (rxData.buttons & 0x01) {
      // Arm drone - add minimum throttle
      if (baseThrottle < 1150) baseThrottle = 1150;
    } else {
      // Disarm drone
      baseThrottle = 1000;
    }
  }
}

void readMPUData() {
  // Read raw data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  // Apply calibration offsets
  float accelX = (ax / 16384.0) - accelX_offset;
  float accelY = (ay / 16384.0) - accelY_offset;
  float accelZ = (az / 16384.0) - accelZ_offset;
  
  float gyroX = (gx / 131.0) - gyroX_offset;
  float gyroY = (gy / 131.0) - gyroY_offset;
  float gyroZ = (gz / 131.0) - gyroZ_offset;
  
  // Calculate angles from accelerometer
  float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;
  
  // Complementary filter
  compAngleX = alpha * (compAngleX + gyroX * dt) + (1 - alpha) * accelAngleX;
  compAngleY = alpha * (compAngleY + gyroY * dt) + (1 - alpha) * accelAngleY;
  
  // Update current angles
  roll = compAngleX;
  pitch = compAngleY;
  yaw += gyroZ * dt; // Integrate gyro for yaw
}

void calculatePID() {
  // Calculate errors
  rollError = rollSetpoint - roll;
  pitchError = pitchSetpoint - pitch;
  yawError = yawSetpoint - yaw;
  
  // Calculate integral terms (with anti-windup)
  rollIntegral += rollError * dt;
  pitchIntegral += pitchError * dt;
  yawIntegral += yawError * dt;
  
  // Constrain integrals
  rollIntegral = constrain(rollIntegral, -100, 100);
  pitchIntegral = constrain(pitchIntegral, -100, 100);
  yawIntegral = constrain(yawIntegral, -100, 100);
  
  // Calculate derivative terms
  float rollDerivative = (rollError - rollPrevError) / dt;
  float pitchDerivative = (pitchError - pitchPrevError) / dt;
  float yawDerivative = (yawError - yawPrevError) / dt;
  
  // Calculate PID outputs
  float rollPID = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;
  float pitchPID = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
  float yawPID = Kp * yawError + Ki * yawIntegral + Kd * yawDerivative;
  
  // Update previous errors
  rollPrevError = rollError;
  pitchPrevError = pitchError;
  yawPrevError = yawError;
  
  // Store PID outputs
  roll = constrain(rollPID, -400, 400);
  pitch = constrain(pitchPID, -400, 400);
  yaw = constrain(yawPID, -400, 400);
}

void mixMotors() {
  // Convert throttle from microseconds to 0-255 range
  int throttle = constrain(map(baseThrottle, 1000, 2000, 0, 255), 0, 255);
  
  // Mix PID outputs with throttle (X configuration)
  motorFL = throttle - roll - pitch + yaw;
  motorFR = throttle + roll - pitch - yaw;
  motorBL = throttle - roll + pitch - yaw;
  motorBR = throttle + roll + pitch + yaw;
  
  // Constrain motor values
  motorFL = constrain(motorFL, 0, 255);
  motorFR = constrain(motorFR, 0, 255);
  motorBL = constrain(motorBL, 0, 255);
  motorBR = constrain(motorBR, 0, 255);
}

void writeMotors() {
  // Only write if throttle is above minimum
  if (baseThrottle > 1100) {
    analogWrite(MOTOR_FL, motorFL);
    analogWrite(MOTOR_FR, motorFR);
    analogWrite(MOTOR_BL, motorBL);
    analogWrite(MOTOR_BR, motorBR);
  } else {
    // Stop all motors
    analogWrite(MOTOR_FL, 0);
    analogWrite(MOTOR_FR, 0);
    analogWrite(MOTOR_BL, 0);
    analogWrite(MOTOR_BR, 0);
  }
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  digitalWrite(BUZZER_PIN, HIGH);
  
  int numSamples = 1000;
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;
  
  for (int i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    sumAx += ax / 16384.0;
    sumAy += ay / 16384.0;
    sumAz += az / 16384.0;
    
    sumGx += gx / 131.0;
    sumGy += gy / 131.0;
    sumGz += gz / 131.0;
    
    delay(1);
  }
  
  accelX_offset = sumAx / numSamples;
  accelY_offset = sumAy / numSamples;
  accelZ_offset = (sumAz / numSamples) - 1.0; // Remove gravity
  
  gyroX_offset = sumGx / numSamples;
  gyroY_offset = sumGy / numSamples;
  gyroZ_offset = sumGz / numSamples;
  
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Calibration complete!");
}

void safetyCheck() {
  static unsigned long lastRadioTime = 0;
  
  // Check for radio signal loss
  if (millis() - lastRadioTime > 100) {
    // No radio signal for 100ms
    baseThrottle = 1000; // Cut throttle
    writeMotors();
    buzzerBeep(1);
  } else {
    lastRadioTime = millis();
  }
  
  // Check for extreme angles (crash detection)
  if (abs(roll) > 45 || abs(pitch) > 45) {
    baseThrottle = 1000;
    writeMotors();
    buzzerBeep(5);
  }
}

void buzzerBeep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(100);
  }
}