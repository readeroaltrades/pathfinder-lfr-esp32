# Pathfinder LFR

An autonomous PID-controlled line follower robot built with ESP32, optimized for competitive racing with intelligent sensor fusion and adaptive control algorithms.

![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32-green.svg)
![Framework](https://img.shields.io/badge/framework-Arduino-00979D.svg)

---

## 📋 Table of Contents

-   [Overview](#overview)
-   [Hardware Components](#hardware-components)
-   [Software Features](#software-features)
-   [Pin Configuration](#pin-configuration)
-   [Getting Started](#getting-started)
-   [PID Tuning Guide](#pid-tuning-guide)
-   [Chassis Specifications](#chassis-specifications)
-   [Troubleshooting](#troubleshooting)
-   [Performance Optimization](#performance-optimization)
-   [Competition Compliance](#competition-compliance)

---

## 🎯 Overview

Pathfinder is a competition-ready line follower robot that uses **PID control** to autonomously navigate black lines on white surfaces. The robot features optimized motor control, intelligent sensor processing, and robust error handling for curves, gaps, and intersections.

### Core Capabilities

-   **Precise Line Tracking**: PID algorithm with integral windup protection
-   **Adaptive Speed Control**: Dynamic motor speed adjustment (50-220 PWM)
-   **Smart Sensor Logic**: 3-sensor array with 7 position states
-   **Built-in Calibration**: Automated sensor threshold detection
-   **Real-time Debugging**: Comprehensive serial monitoring at 100ms intervals

---

## 🔧 Hardware Components

| Component           | Specification              | Quantity |
| ------------------- | -------------------------- | -------- |
| **Microcontroller** | ESP32 DevKit               | 1        |
| **Motors**          | N20 1000RPM Gear Motors    | 2        |
| **Motor Driver**    | L298N H-Bridge             | 1        |
| **Line Sensors**    | TCRT5000 IR Sensors        | 3        |
| **Power Supply**    | 3× 18650 Batteries (11.1V) | 1 pack   |
| **Chassis**         | PVC Sheet (9.45"×7.87")    | 1        |
| **Caster Wheel**    | Free-rotating rear support | 1        |

### Power Requirements

-   **Motor Supply**: 11.1V (3S Li-ion)
-   **Logic Supply**: 3.3V (regulated from battery)
-   **Peak Current**: ~2A (both motors under load)

---

## ⚡ Software Features

### 1. **PID Control System**

```cpp
Kp = 25.0  // Proportional gain (primary correction)
Ki = 0.0   // Integral gain (eliminates steady-state drift)
Kd = 8.0   // Derivative gain (dampens oscillations)
```

**Control Loop Frequency**: 100 Hz (10ms cycle time)

### 2. **Integral Windup Protection**

Prevents integral term accumulation during line loss or sharp turns:

```cpp
integral = constrain(integral, -100.0, +100.0)
```

### 3. **Hardware PWM via LEDC**

-   **Frequency**: 5 kHz (optimized for N20 motors with L298N)
-   **Resolution**: 8-bit (0-255)
-   **Channels**: Independent left/right motor control

### 4. **Intelligent Line Detection**

| Sensor State (L-M-R) | Error     | Robot Action                   |
| -------------------- | --------- | ------------------------------ |
| `1-0-0`              | -2        | Hard left turn                 |
| `1-1-0`              | -1        | Slight left correction         |
| `0-1-0`              | 0         | Centered - go straight         |
| `0-1-1`              | +1        | Slight right correction        |
| `0-0-1`              | +2        | Hard right turn                |
| `1-1-1`              | 0         | Wide line/intersection         |
| `0-0-0`              | lastError | Line lost - maintain direction |

### 5. **Automatic Calibration**

```cpp
calibrateSensors();  // Measures black/white values
```

-   Samples 50 readings per surface
-   Calculates optimal threshold automatically
-   Compensates for ambient lighting

### 6. **Real-time Debug Output**

```
Sensors[L:2800 M:500 R:2900] | Err:0 | Corr:0.0 | Speed[L:180 R:180]
```

Shows sensor readings, position error, PID correction, and motor speeds.

---

## 📌 Pin Configuration

### Sensor Pins (Analog)

```cpp
LEFT_SENSOR   = 34  // ADC1_CH6
MID_SENSOR    = 35  // ADC1_CH7
RIGHT_SENSOR  = 32  // ADC1_CH4
```

### Motor Control (L298N)

```cpp
// Left Motor
ENA = 33  // PWM (LEDC Channel 0)
IN1 = 25  // Direction A
IN2 = 26  // Direction B

// Right Motor
ENB = 27  // PWM (LEDC Channel 1)
IN3 = 14  // Direction A
IN4 = 12  // Direction B
```

### Wiring Diagram

```
ESP32          L298N          Motors
-----          ------         ------
 33  ────────→  ENA  ────────→ Left Motor +
 25  ────────→  IN1           Left Motor -
 26  ────────→  IN2
 27  ────────→  ENB  ────────→ Right Motor +
 14  ────────→  IN3           Right Motor -
 12  ────────→  IN4
GND  ────────→  GND
11.1V ───────→  12V/VCC
```

---

## 🚀 Getting Started

### Prerequisites

-   [PlatformIO IDE](https://platformio.org/) or Arduino IDE with ESP32 support
-   USB cable for programming
-   Charged 3S Li-ion battery pack

### Installation

1. **Clone the repository**

```bash
git clone https://github.com/yourusername/pathfinder-lfr-esp32.git
cd pathfinder-lfr-esp32
```

2. **Open in PlatformIO**

```bash
pio run
```

3. **Configure Upload Port** (in `platformio.ini`)

```ini
upload_port = COM5     # Windows
# upload_port = /dev/ttyUSB0  # Linux
```

4. **Upload firmware**

```bash
pio run --target upload
```

### First-Time Calibration

1. **Uncomment calibration function** in `main.cpp`:

```cpp
void setup() {
  // ...
  calibrateSensors();  // Enable this line
  // ...
}
```

2. **Upload and open Serial Monitor**

```bash
pio device monitor
```

3. **Follow calibration prompts**:

    - Place sensors over WHITE surface → wait 3 seconds
    - Place sensors over BLACK line → wait 3 seconds
    - Note the recommended threshold value

4. **Update threshold in code**:

```cpp
int threshold = 1600;  // Use recommended value from calibration
```

5. **Re-upload** with calibration disabled

---

## 🎛️ PID Tuning Guide

### Step 1: Tune Kp (Proportional)

Start with `Ki = 0` and `Kd = 0`

1. Set `Kp = 10`
2. Increase gradually until robot follows line with slight wobble
3. Reduce Kp by 20% for stability

**Symptoms:**

-   Too low → Sluggish, misses turns
-   Too high → Violent oscillations

### Step 2: Tune Kd (Derivative)

1. Keep Kp at optimal value
2. Start `Kd = 5`
3. Increase until wobble is minimized
4. Stop if robot becomes too slow to respond

**Symptoms:**

-   Too low → Still oscillating
-   Too high → Sluggish cornering

### Step 3: Tune Ki (Integral) - Optional

Only add if robot consistently drifts off-center on straight lines.

1. Start with `Ki = 0.1`
2. Increase slowly (max 0.5)
3. Watch for integral windup issues

**Current Recommended Values:**

```cpp
Kp = 25.0  // Aggressive correction
Ki = 0.0   // Disabled (not needed for most tracks)
Kd = 8.0   // Smooth damping
```

### Speed Adjustment

```cpp
baseSpeed = 180   // Cruising speed (start conservative)
maxSpeed  = 220   // Upper limit (allows PID headroom)
minSpeed  = 50    // Prevents motor stall
```

**Tuning Tips:**

-   Start at `baseSpeed = 150` for testing
-   Increase gradually until performance degrades
-   Keep `maxSpeed = baseSpeed + 40` for safety margin

---

## 📐 Chassis Specifications

### Competition Limits

| Parameter | Maximum  | Actual     |
| --------- | -------- | ---------- |
| Width     | 9.84 in  | 9.45 in    |
| Length    | 9.84 in  | 7.87 in    |
| Height    | 7.09 in  | ≤ 7.09 in  |
| Weight    | 2.65 lbs | ≤ 2.65 lbs |

### Physical Layout

| Feature                            | Measurement    |
| ---------------------------------- | -------------- |
| Wheel spacing (center-to-center)   | 5.51 in        |
| Wheelbase (front–rear axle)        | 3.94 in        |
| Caster offset (behind rear axle)   | 0.59–0.98 in   |
| Sensor bar offset (ahead of front) | 0.39–0.71 in   |
| Sensor ground clearance            | 0.08–0.16 in   |
| Battery area                       | 2.76 × 1.57 in |
| ESP32 mounting area                | 1.97 × 1.18 in |

### Material

-   **Base Plate**: ⅛" (3mm) PVC sheet
-   **Thickness**: 0.12–0.16 in
-   **Finish**: Deburred edges, rounded corners (⅛" radius)

### Center of Gravity

-   **Battery**: Centered laterally, 0.4–0.8 in forward of motor axis
-   **ESP32**: Centered on chassis
-   **Weight Distribution**: ~45% front, 55% rear

---

## 🔍 Troubleshooting

### Robot Oscillates/Wobbles

**Cause**: Kp too high or Kd too low  
**Solution**:

-   Decrease Kp by 20%
-   Increase Kd by 2-3 points

### Misses Sharp Turns

**Cause**: Speed too high or correction insufficient  
**Solution**:

-   Reduce `baseSpeed` to 150-160
-   Increase `maxSpeed` to 240
-   Increase Kp for more aggressive steering

### Motors Stall on Turns

**Cause**: `minSpeed` too low, especially with weak batteries  
**Solution**:

-   Increase `minSpeed` to 80-100
-   Check battery voltage (should be > 10.5V)

### Sensors Not Detecting Line

**Cause**: Incorrect threshold or sensor height  
**Solution**:

1. Run calibration routine
2. Check sensor-to-ground clearance (0.08-0.16 in)
3. Verify sensor readings in Serial Monitor
4. Adjust threshold if needed

### Inverted Sensor Logic

Some TCRT5000 modules output HIGH on black line.  
**Solution**: Change threshold comparison:

```cpp
bool leftOnLine = (leftVal > threshold);  // For inverted sensors
```

### Random Behavior / Noise

**Cause**: Power supply noise or loose connections  
**Solution**:

-   Add 100µF capacitor across motor terminals
-   Check all ground connections
-   Secure sensor cables (< 3 in length)

---

## ⚡ Performance Optimization

### Speed vs. Accuracy Trade-offs

| Track Type    | Base Speed | Max Speed | Kp  | Kd  |
| ------------- | ---------- | --------- | --- | --- |
| Gentle curves | 200        | 240       | 20  | 6   |
| Sharp turns   | 160        | 200       | 28  | 10  |
| Mixed track   | 180        | 220       | 25  | 8   |

### Advanced Techniques

1. **Adaptive Speed Control** (TODO)

    - Reduce speed proportional to error magnitude
    - `speed = baseSpeed - (abs(error) * speedFactor)`

2. **Look-ahead Sensor** (Requires 5-sensor array)

    - Use outer sensors to predict upcoming turns
    - Pre-emptively adjust speed

3. **Encoder Integration** (TODO)
    - Track wheel rotations for gap recovery
    - Precise distance measurement for checkpoints

---

## 🏁 Competition Compliance

### Pre-Competition Checklist

-   [ ] **Dimensions verified**: ≤ 9.84" × 9.84" × 7.09"
-   [ ] **Weight**: ≤ 2.65 lbs
-   [ ] **Kill switch**: Accessible from top/side, clearly labeled
-   [ ] **Loose parts**: All components secured (no loose wires)
-   [ ] **Sharp edges**: All corners rounded, bolts flush/recessed
-   [ ] **Battery**: Secure mounting, proper polarity
-   [ ] **Calibration**: Fresh calibration on competition track
-   [ ] **Test runs**: 3+ successful completions on practice track
-   [ ] **Backup code**: Previous stable version saved

### Track Specifications (Typical)

-   **Line Width**: 0.75–1 in
-   **Line Color**: Black on white
-   **Gaps**: Up to 15 cm
-   **Checkpoints**: Wide black zones (detect with all sensors)
-   **Starting Zone**: 30×30 cm black square

### Autonomous Operation Rules

-   No remote control during runs
-   Robot must self-start after calibration
-   3-minute time limit per run
-   Scoring: Checkpoints × Speed bonus

---

## 📁 Project Structure

```
pathfinder-lfr-esp32/
├── src/
│   └── main.cpp              # Main firmware
├── include/
│   └── README                # Library headers (if needed)
├── platformio.ini            # PlatformIO configuration
├── README.md                 # This file
└── LICENSE
```

---

## 🛠️ Development Roadmap

-   [x] PID control implementation
-   [x] Integral windup protection
-   [x] Hardware PWM via LEDC
-   [x] Automatic calibration routine
-   [x] Real-time debug monitoring
-   [ ] BMS integration for battery protection
-   [ ] Hardware kill switch implementation
-   [ ] 5-sensor array support
-   [ ] Adaptive speed control
-   [ ] Checkpoint detection logic
-   [ ] Gap recovery with timing
-   [ ] EEPROM PID storage
-   [ ] Web-based tuning interface

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📧 Contact

Project Link: [https://github.com/yourusername/pathfinder-lfr-esp32](https://github.com/yourusername/pathfinder-lfr-esp32)

---

## 🙏 Acknowledgments

-   ESP32 Arduino Core by Espressif
-   PlatformIO for seamless development experience
-   TCRT5000 sensor reference designs
-   PID control theory resources

---
