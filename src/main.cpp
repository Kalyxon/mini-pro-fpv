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
#define LED_PIN 13  // Built-in LED

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

// PID Gains (Tune these for your drone)
float Kp_roll = 1.2, Ki_roll = 0.05, Kd_roll = 0.15;
float Kp_pitch = 1.2, Ki_pitch = 0.05, Kd_pitch = 0.15;
float Kp_yaw = 2.0, Ki_yaw = 0.02, Kd_yaw = 0.05;

// PID Variables
float rollError, pitchError, yawError;
float rollPrevError = 0, pitchPrevError = 0, yawPrevError = 0;
float rollIntegral = 0, pitchIntegral = 0, yawIntegral = 0;
float rollOutput = 0, pitchOutput = 0, yawOutput = 0;

// Motor Control
int motorFL = 0, motorFR = 0, motorBL = 0, motorBR = 0;
int baseThrottle = 1000;  // Start at minimum throttle (1000-2000 range)
int minThrottle = 1150;   // Minimum throttle to start motors
int maxThrottle = 1850;   // Maximum throttle limit

// Control structure for radio communication
struct ControlData {
  byte throttle;
  byte yaw;
  byte pitch;
  byte roll;
  byte buttons;
};

ControlData rxData;

// State variables
bool isArmed = false;
bool radioConnected = false;
unsigned long lastRadioSignal = 0;
unsigned long armTime = 0;

// Timing variables
unsigned long prevTime = 0;
float dt = 0.0;
unsigned long loopTimer = 0;
const int LOOP_FREQUENCY = 250;  // Hz
const float LOOP_TIME = 1000000.0 / LOOP_FREQUENCY;  // microseconds

// MPU Calibration offsets
float accelX_offset = 0, accelY_offset = 0, accelZ_offset = 0;
float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;

// Angle variables
float angleX = 0, angleY = 0, angleZ = 0;
float compAngleX = 0, compAngleY = 0;
float alpha = 0.96;  // Complementary filter constant

// Motor mixing configuration (X configuration)
const float MIX_MATRIX[4][3] = {
  { -1.0, -1.0,  1.0 },  // Motor FL: -roll, -pitch, +yaw
  {  1.0, -1.0, -1.0 },  // Motor FR: +roll, -pitch, -yaw
  { -1.0,  1.0, -1.0 },  // Motor BL: -roll, +pitch, -yaw
  {  1.0,  1.0,  1.0 }   // Motor BR: +roll, +pitch, +yaw
};

// ==================== FUNCTION DECLARATIONS ====================
void readRadioData();
void readMPUData();
void calculatePID();
void mixMotors();
void writeMotors();
void calibrateMPU();
void safetyCheck();
void buzzerBeep(int times);
void ledBlink(int times, int duration);
void armDrone();
void disarmDrone();
void printTelemetry();
void emergencyStop();
void handleSerialCommand(char command);
void testMotors();

// ==================== SETUP ====================
void setup() {
  // Initialize Serial
  Serial.begin(115200);
  Serial.println("=== C++ Drone Controller Initialization ===");
  
  // Initialize pins
  pinMode(MOTOR_FL, OUTPUT);
  pinMode(MOTOR_FR, OUTPUT);
  pinMode(MOTOR_BL, OUTPUT);
  pinMode(MOTOR_BR, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Ensure motors are stopped
  analogWrite(MOTOR_FL, 0);
  analogWrite(MOTOR_FR, 0);
  analogWrite(MOTOR_BL, 0);
  analogWrite(MOTOR_BR, 0);
  
  // Startup sequence
  Serial.println("Starting up...");
  buzzerBeep(3);
  ledBlink(3, 200);
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize MPU6050
  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
    buzzerBeep(2);
  } else {
    Serial.println("MPU6050 connection failed!");
    while(1) {
      buzzerBeep(1);
      ledBlink(1, 500);
      delay(500);
    }
  }
  
  // Calibrate MPU6050
  calibrateMPU();
  
  // Initialize NRF24L01
  Serial.println("Initializing NRF24L01...");
  if (radio.begin()) {
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MAX);      // Use MAX for better range
    radio.setDataRate(RF24_250KBPS);    // Lower data rate = more reliable
    radio.setChannel(100);              // Avoid common WiFi channels
    radio.setRetries(15, 15);           // Max retries
    radio.setCRCLength(RF24_CRC_16);    // Better error detection
    radio.startListening();
    
    Serial.println("NRF24L01 initialized successfully");
    Serial.print("Listening on address: ");
    for (int i = 0; i < 5; i++) {
      Serial.print(address[i], HEX);
    }
    Serial.println();
    
    buzzerBeep(4);
    ledBlink(2, 100);
  } else {
    Serial.println("NRF24L01 initialization failed!");
    while(1) {
      buzzerBeep(2);
      ledBlink(2, 250);
      delay(1000);
    }
  }
  
  // Wait for transmitter connection
  Serial.println("Waiting for transmitter...");
  unsigned long startTime = millis();
  while (millis() - startTime < 5000) {
    if (radio.available()) {
      radioConnected = true;
      break;
    }
    ledBlink(1, 100);
    delay(100);
  }
  
  if (radioConnected) {
    Serial.println("Transmitter connected!");
    buzzerBeep(1);
  } else {
    Serial.println("No transmitter found. Starting anyway...");
  }
  
  Serial.println("\n=== Drone Ready ===");
  Serial.println("Commands: ");
  Serial.println("  'a' - Arm drone");
  Serial.println("  'd' - Disarm drone");
  Serial.println("  's' - Print status");
  Serial.println("  'c' - Calibrate MPU");
  Serial.println("  'e' - Emergency stop");
  Serial.println("  't' - Test motors");
  Serial.println("========================\n");
  
  delay(1000);  // Final stabilization
  loopTimer = micros();
}

// ==================== MAIN LOOP ====================
void loop() {
  // Fixed frequency loop
  unsigned long currentTime = micros();
  if (currentTime - loopTimer >= LOOP_TIME) {
    dt = (currentTime - loopTimer) / 1000000.0;
    loopTimer = currentTime;
    
    // Read serial commands
    if (Serial.available()) {
      char command = Serial.read();
      handleSerialCommand(command);
    }
    
    // Read radio data
    readRadioData();
    
    // Read MPU data
    readMPUData();
    
    // Calculate PID if armed
    if (isArmed) {
      calculatePID();
      mixMotors();
      writeMotors();
    }
    
    // Safety checks
    safetyCheck();
    
    // Print telemetry every 100 loops (about 0.4 seconds)
    static int telemetryCounter = 0;
    if (telemetryCounter++ >= 100) {
      telemetryCounter = 0;
      printTelemetry();
    }
    
    // Blink LED to show activity
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink >= (isArmed ? 100 : 500)) {
      lastBlink = millis();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
  }
}

// ==================== SERIAL COMMAND HANDLER ====================
void handleSerialCommand(char command) {
  switch (command) {
    case 'a':
    case 'A':
      armDrone();
      break;
      
    case 'd':
    case 'D':
      disarmDrone();
      break;
      
    case 's':
    case 'S':
      printTelemetry();
      break;
      
    case 'c':
    case 'C':
      calibrateMPU();
      break;
      
    case 'e':
    case 'E':
      emergencyStop();
      break;
      
    case 't':
    case 'T':
      // Test motors
      Serial.println("Testing motors...");
      testMotors();
      break;
      
    default:
      Serial.println("Unknown command. Use: a=arm, d=disarm, s=status, c=calibrate, e=emergency, t=test");
      break;
  }
}

// ==================== RADIO DATA READING ====================
void readRadioData() {
  if (radio.available()) {
    radio.read(&rxData, sizeof(rxData));
    radioConnected = true;
    lastRadioSignal = millis();
    
    // Map receiver values
    baseThrottle = map(rxData.throttle, 0, 255, 1000, maxThrottle);
    rollSetpoint = map(rxData.roll, 0, 255, -20.0, 20.0);
    pitchSetpoint = map(rxData.pitch, 0, 255, -20.0, 20.0);
    yawSetpoint = map(rxData.yaw, 0, 255, -100.0, 100.0);
    
    // Arm/disarm with button
    if (rxData.buttons & 0x01) {  // Button 1
      if (!isArmed && baseThrottle < minThrottle + 50) {
        armDrone();
      }
    } else {
      if (isArmed) {
        disarmDrone();
      }
    }
    
    // Additional button functions (optional)
    if (rxData.buttons & 0x02) {  // Button 2
      // Optional: Altitude hold, auto-level, etc.
    }
  } else {
    // Check for signal loss
    if (millis() - lastRadioSignal > 200) {
      radioConnected = false;
    }
  }
}

// ==================== MPU6050 DATA READING ====================
void readMPUData() {
  static float gyroX_prev = 0, gyroY_prev = 0, gyroZ_prev = 0;
  
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
  
  // Apply low-pass filter to gyro (optional)
  float filterFactor = 0.3;
  gyroX = filterFactor * gyroX + (1 - filterFactor) * gyroX_prev;
  gyroY = filterFactor * gyroY + (1 - filterFactor) * gyroY_prev;
  gyroZ = filterFactor * gyroZ + (1 - filterFactor) * gyroZ_prev;
  
  gyroX_prev = gyroX;
  gyroY_prev = gyroY;
  gyroZ_prev = gyroZ;
  
  // Calculate angles from accelerometer
  float accelAngleX = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;
  
  // Complementary filter
  compAngleX = alpha * (compAngleX + gyroX * dt) + (1.0 - alpha) * accelAngleX;
  compAngleY = alpha * (compAngleY + gyroY * dt) + (1.0 - alpha) * accelAngleY;
  
  // Update current angles
  roll = compAngleX;
  pitch = compAngleY;
  yaw += gyroZ * dt;
  
  // Keep yaw between -180 and 180 degrees
  if (yaw > 180) yaw -= 360;
  if (yaw < -180) yaw += 360;
}

// ==================== PID CALCULATION ====================
void calculatePID() {
  // Calculate errors
  rollError = rollSetpoint - roll;
  pitchError = pitchSetpoint - pitch;
  yawError = yawSetpoint - yaw;
  
  // Calculate integral terms with anti-windup
  rollIntegral += rollError * dt;
  pitchIntegral += pitchError * dt;
  yawIntegral += yawError * dt;
  
  // Limit integrals
  rollIntegral = constrain(rollIntegral, -200, 200);
  pitchIntegral = constrain(pitchIntegral, -200, 200);
  yawIntegral = constrain(yawIntegral, -200, 200);
  
  // Calculate derivatives
  float rollDerivative = (rollError - rollPrevError) / dt;
  float pitchDerivative = (pitchError - pitchPrevError) / dt;
  float yawDerivative = (yawError - yawPrevError) / dt;
  
  // Apply low-pass filter to derivatives
  float dFilter = 0.7;
  static float rollDerivFiltered = 0, pitchDerivFiltered = 0, yawDerivFiltered = 0;
  rollDerivFiltered = dFilter * rollDerivFiltered + (1 - dFilter) * rollDerivative;
  pitchDerivFiltered = dFilter * pitchDerivFiltered + (1 - dFilter) * pitchDerivative;
  yawDerivFiltered = dFilter * yawDerivFiltered + (1 - dFilter) * yawDerivative;
  
  // Calculate PID outputs
  rollOutput = Kp_roll * rollError + Ki_roll * rollIntegral + Kd_roll * rollDerivFiltered;
  pitchOutput = Kp_pitch * pitchError + Ki_pitch * pitchIntegral + Kd_pitch * pitchDerivFiltered;
  yawOutput = Kp_yaw * yawError + Ki_yaw * yawIntegral + Kd_yaw * yawDerivFiltered;
  
  // Limit PID outputs
  rollOutput = constrain(rollOutput, -300, 300);
  pitchOutput = constrain(pitchOutput, -300, 300);
  yawOutput = constrain(yawOutput, -150, 150);
  
  // Update previous errors
  rollPrevError = rollError;
  pitchPrevError = pitchError;
  yawPrevError = yawError;
}

// ==================== MOTOR MIXING ====================
void mixMotors() {
  // Convert throttle from microseconds to 0-255 range
  int throttle = constrain(map(baseThrottle, 1000, maxThrottle, 0, 255), 0, 255);
  
  // Apply motor mixing matrix
  motorFL = throttle + (MIX_MATRIX[0][0] * rollOutput + MIX_MATRIX[0][1] * pitchOutput + MIX_MATRIX[0][2] * yawOutput);
  motorFR = throttle + (MIX_MATRIX[1][0] * rollOutput + MIX_MATRIX[1][1] * pitchOutput + MIX_MATRIX[1][2] * yawOutput);
  motorBL = throttle + (MIX_MATRIX[2][0] * rollOutput + MIX_MATRIX[2][1] * pitchOutput + MIX_MATRIX[2][2] * yawOutput);
  motorBR = throttle + (MIX_MATRIX[3][0] * rollOutput + MIX_MATRIX[3][1] * pitchOutput + MIX_MATRIX[3][2] * yawOutput);
  
  // Constrain motor values
  motorFL = constrain(motorFL, 0, 255);
  motorFR = constrain(motorFR, 0, 255);
  motorBL = constrain(motorBL, 0, 255);
  motorBR = constrain(motorBR, 0, 255);
}

// ==================== MOTOR CONTROL ====================
void writeMotors() {
  if (isArmed && baseThrottle > minThrottle) {
    analogWrite(MOTOR_FL, motorFL);
    analogWrite(MOTOR_FR, motorFR);
    analogWrite(MOTOR_BL, motorBL);
    analogWrite(MOTOR_BR, motorBR);
  } else {
    analogWrite(MOTOR_FL, 0);
    analogWrite(MOTOR_FR, 0);
    analogWrite(MOTOR_BL, 0);
    analogWrite(MOTOR_BR, 0);
  }
}

// ==================== MPU CALIBRATION ====================
void calibrateMPU() {
  Serial.println("\n=== MPU6050 Calibration ===");
  Serial.println("Keep the drone perfectly still and level!");
  Serial.println("Calibration starts in 3 seconds...");
  
  buzzerBeep(3);
  delay(3000);
  
  digitalWrite(BUZZER_PIN, HIGH);
  Serial.println("Calibrating...");
  
  int numSamples = 2000;
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
    
    if (i % 200 == 0) {
      Serial.print(".");
    }
    delay(1);
  }
  
  accelX_offset = sumAx / numSamples;
  accelY_offset = sumAy / numSamples;
  accelZ_offset = (sumAz / numSamples) - 1.0;  // Remove gravity
  
  gyroX_offset = sumGx / numSamples;
  gyroY_offset = sumGy / numSamples;
  gyroZ_offset = sumGz / numSamples;
  
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("\nCalibration complete!");
  
  Serial.print("Accel offsets - X: ");
  Serial.print(accelX_offset, 4);
  Serial.print(" Y: ");
  Serial.print(accelY_offset, 4);
  Serial.print(" Z: ");
  Serial.println(accelZ_offset, 4);
  
  Serial.print("Gyro offsets - X: ");
  Serial.print(gyroX_offset, 4);
  Serial.print(" Y: ");
  Serial.print(gyroY_offset, 4);
  Serial.print(" Z: ");
  Serial.println(gyroZ_offset, 4);
  
  buzzerBeep(1);
}

// ==================== SAFETY CHECKS ====================
void safetyCheck() {
  // Radio signal loss
  if (!radioConnected) {
    if (isArmed) {
      Serial.println("Radio signal lost! Disarming...");
      disarmDrone();
    }
    return;
  }
  
  // Check for extreme angles (crash detection)
  if (abs(roll) > 60.0 || abs(pitch) > 60.0) {
    Serial.println("Extreme angle detected! Emergency stop!");
    emergencyStop();
    return;
  }
  
  // Auto-disarm if throttle too low for too long
  if (isArmed && baseThrottle < minThrottle) {
    static unsigned long lowThrottleTime = 0;
    if (lowThrottleTime == 0) {
      lowThrottleTime = millis();
    } else if (millis() - lowThrottleTime > 3000) {  // 3 seconds
      Serial.println("Auto-disarming due to low throttle");
      disarmDrone();
      lowThrottleTime = 0;
    }
  } else {
    static unsigned long lowThrottleTime = 0;
    lowThrottleTime = 0;
  }
}

// ==================== DRONE ARMING ====================
void armDrone() {
  if (isArmed) return;
  
  // Safety checks before arming
  if (!radioConnected) {
    Serial.println("Cannot arm: No radio signal!");
    buzzerBeep(3);
    return;
  }
  
  if (baseThrottle > minThrottle + 50) {
    Serial.println("Cannot arm: Throttle too high!");
    buzzerBeep(4);
    return;
  }
  
  if (abs(roll) > 15 || abs(pitch) > 15) {
    Serial.println("Cannot arm: Drone not level!");
    buzzerBeep(5);
    return;
  }
  
  isArmed = true;
  armTime = millis();
  
  Serial.println("\n*** DRONE ARMED ***");
  Serial.println("WARNING: Motors are now live!");
  Serial.println("Keep clear of propellers!\n");
  
  buzzerBeep(2);
  ledBlink(10, 50);
  
  // Reset PID integrals
  rollIntegral = 0;
  pitchIntegral = 0;
  yawIntegral = 0;
  rollPrevError = 0;
  pitchPrevError = 0;
  yawPrevError = 0;
}

// ==================== DRONE DISARMING ====================
void disarmDrone() {
  if (!isArmed) return;
  
  isArmed = false;
  baseThrottle = 1000;
  
  // Stop motors
  analogWrite(MOTOR_FL, 0);
  analogWrite(MOTOR_FR, 0);
  analogWrite(MOTOR_BL, 0);
  analogWrite(MOTOR_BR, 0);
  
  Serial.println("\n*** DRONE DISARMED ***");
  Serial.print("Flight time: ");
  Serial.print((millis() - armTime) / 1000.0, 1);
  Serial.println(" seconds\n");
  
  buzzerBeep(1);
  delay(200);
  buzzerBeep(1);
}

// ==================== EMERGENCY STOP ====================
void emergencyStop() {
  Serial.println("\n!!! EMERGENCY STOP !!!");
  
  isArmed = false;
  baseThrottle = 1000;
  
  // Immediate motor cutoff
  analogWrite(MOTOR_FL, 0);
  analogWrite(MOTOR_FR, 0);
  analogWrite(MOTOR_BL, 0);
  analogWrite(MOTOR_BR, 0);
  
  // Alarm sequence
  for (int i = 0; i < 10; i++) {
    buzzerBeep(1);
    ledBlink(1, 100);
    delay(100);
  }
}

// ==================== BUZZER CONTROL ====================
void buzzerBeep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(80);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < times - 1) delay(80);
  }
}

// ==================== LED CONTROL ====================
void ledBlink(int times, int duration) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(duration);
    digitalWrite(LED_PIN, LOW);
    if (i < times - 1) delay(duration);
  }
}

// ==================== TELEMETRY ====================
void printTelemetry() {
  Serial.println("\n=== TELEMETRY ===");
  Serial.print("Armed: ");
  Serial.print(isArmed ? "YES" : "NO");
  Serial.print(" | Radio: ");
  Serial.println(radioConnected ? "OK" : "LOST");
  
  Serial.print("Angles - Roll: ");
  Serial.print(roll, 1);
  Serial.print("° | Pitch: ");
  Serial.print(pitch, 1);
  Serial.print("° | Yaw: ");
  Serial.print(yaw, 1);
  Serial.println("°");
  
  Serial.print("Setpoints - R: ");
  Serial.print(rollSetpoint, 1);
  Serial.print(" | P: ");
  Serial.print(pitchSetpoint, 1);
  Serial.print(" | Y: ");
  Serial.println(yawSetpoint, 1);
  
  Serial.print("Throttle: ");
  Serial.print(baseThrottle);
  Serial.print(" | PID Out - R: ");
  Serial.print(rollOutput, 0);
  Serial.print(" | P: ");
  Serial.print(pitchOutput, 0);
  Serial.print(" | Y: ");
  Serial.println(yawOutput, 0);
  
  Serial.print("Motors - FL: ");
  Serial.print(motorFL);
  Serial.print(" | FR: ");
  Serial.print(motorFR);
  Serial.print(" | BL: ");
  Serial.print(motorBL);
  Serial.print(" | BR: ");
  Serial.println(motorBR);
  
  Serial.print("Loop time: ");
  Serial.print(dt * 1000, 2);
  Serial.print("ms | Freq: ");
  Serial.print(1.0 / dt, 0);
  Serial.println("Hz");
  Serial.println("==================\n");
}

// ==================== MOTOR TEST ====================
void testMotors() {
  disarmDrone();
  
  Serial.println("Testing each motor (1 second each)...");
  
  int motors[4] = {MOTOR_FL, MOTOR_FR, MOTOR_BL, MOTOR_BR};
  const char* motorNames[4] = {"Front Left", "Front Right", "Back Left", "Back Right"};
  
  for (int i = 0; i < 4; i++) {
    Serial.print("Testing motor ");
    Serial.println(motorNames[i]);
    
    buzzerBeep(1);
    
    // Ramp up
    for (int pwm = 0; pwm <= 100; pwm += 10) {
      analogWrite(motors[i], pwm);
      delay(50);
    }
    
    // Hold at 100 for 1 second
    delay(1000);
    
    // Ramp down
    for (int pwm = 100; pwm >= 0; pwm -= 10) {
      analogWrite(motors[i], pwm);
      delay(50);
    }
    
    analogWrite(motors[i], 0);
    delay(500);
  }
  
  Serial.println("Motor test complete!");
  buzzerBeep(2);
}