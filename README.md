# 🚁 Mini Pro Quadcopter Flight Controller

![Drone Banner](https://img.shields.io/badge/Drone-Flight%20Controller-blue)
![PlatformIO](https://img.shields.io/badge/Platform-PlatformIO-green)
![Arduino](https://img.shields.io/badge/Board-Arduino-blue)
![License](https://img.shields.io/badge/License-MIT-yellow)

<div align="center">
  
  ✨ **Stable Flight Controller with MPU6050 + NRF24L01** ✨
  
</div>

## 📋 Overview

A complete flight controller for DIY quadcopters using Arduino, MPU6050 gyroscope/accelerometer, and NRF24L01 radio module for wireless control.

## 🛠️ Hardware Requirements

| Component | Purpose |
|-----------|---------|
| MINI PRO | Main controller |
| MPU6050 | Gyroscope & Accelerometer |
| NRF24L01 | Wireless communication |
| 4x Brushless Motors | Flight propulsion |
| 4x ESCs | Motor speed control |
| Buzzer | Audio feedback |
| LED | Status indicator |

## 🚀 Features

- ✅ **PID Stabilization** - Stable hover control
- ✅ **Radio Control** - Wireless operation up to 100m
- ✅ **Safety Features** - Auto-disarm, crash detection
- ✅ **Serial Interface** - Easy debugging and calibration
- ✅ **Motor Testing** - Individual motor control
- ✅ **Real-time Telemetry** - Monitor flight parameters

## 📁 Project Structure

```
src/
├── main.cpp              # Main flight controller code
├── platformio.ini        # Build configuration
└── lib/                  # Required libraries
    ├── I2Cdevlib-Core
    ├── I2Cdevlib-MPU6050
    └── RF24
```

## ⚡ Quick Start

### 1. Install Dependencies
```bash
# Install PlatformIO extension in VSCode
# Libraries will auto-install from platformio.ini
```

### 2. Upload Code
```bash
# Connect Arduino and upload
platformio run --target upload
```

### 3. Calibrate Sensors
1. Open Serial Monitor (115200 baud)
2. Type `c` and press Enter
3. Keep drone perfectly still during calibration

### 4. Test Motors
1. Type `t` to test each motor (remove props!)
2. Confirm all motors spin correctly

### 5. First Flight
1. Arm drone with transmitter button
2. Slowly increase throttle
3. Make small corrections

## 🎮 Serial Commands

| Command | Function |
|---------|----------|
| `a` | Arm drone |
| `d` | Disarm drone |
| `c` | Calibrate MPU6050 |
| `t` | Test motors |
| `s` | Show telemetry |
| `e` | Emergency stop |

## ⚙️ PID Tuning

```cpp
// Adjust these values in main.cpp
float Kp_roll = 1.2;    // Proportional gain
float Ki_roll = 0.05;   // Integral gain  
float Kd_roll = 0.15;   // Derivative gain
```

**Tuning Tips:**
1. Start with low values
2. Increase P until oscillations
3. Add D to reduce oscillations
4. Add small I for steady hover

## 🔧 Configuration

### Pin Definitions
```cpp
#define MOTOR_FL 6   // Front Left Motor
#define MOTOR_FR 9   // Front Right Motor  
#define MOTOR_BL 5   // Back Left Motor
#define MOTOR_BR 3   // Back Right Motor
#define BUZZER_PIN 8 // Buzzer
#define LED_PIN 13   // Status LED
```

### Radio Settings
```cpp
radio.setPALevel(RF24_PA_MAX);    // Power level
radio.setDataRate(RF24_250KBPS);  // Data rate
radio.setChannel(100);            // Frequency channel
```

## ⚠️ Safety Warnings

- **ALWAYS** remove propellers during testing
- Test in open area away from people
- Keep fire extinguisher nearby
- Monitor battery voltage
- Emergency stop: Throttle down + disarm button

## 📊 Telemetry Output

```
=== TELEMETRY ===
Armed: YES | Radio: OK
Angles - Roll: 2.1° | Pitch: -1.5° | Yaw: 45.2°
Throttle: 1250 | PID Out - R: 12 | P: -8 | Y: 5
Motors - FL: 130 | FR: 125 | BL: 128 | BR: 122
Loop time: 4.02ms | Freq: 249Hz
```

## 🎯 Performance Targets

- **Loop Rate**: 250Hz stable
- **Latency**: <20ms radio response
- **Angle Stability**: ±2° in hover
- **Range**: 50-100m with NRF24L01

## 🤝 Contributing

Found a bug or have a feature request? Feel free to:
1. Fork the repository
2. Create a feature branch
3. Submit a Pull Request

## 📄 License

MIT License - See [LICENSE](LICENSE) for details.

## 🙏 Acknowledgments

- **MPU6050 Library**: Jeff Rowberg's I2Cdevlib
- **NRF24L01**: TMRh20 RF24 library
- **PID Algorithm**: Classic control theory

---

<div align="center">

### 🚀 Happy Flying! 🚀

**Always Fly Responsibly**

*For educational purposes only. Follow local drone regulations.*

</div>

## 📧 Contact

**Project Maintainer**: Kalyxon
**GitHub**: [@Kalxon](https://github.com/Kalyxon)

---

⭐ **If this project helps you, give it a star!** ⭐

