# Pathfinder LFR ESP32 - Technical Documentation

## Overview
High-performance line following robot (LFR) optimized for 100Hz control loop with adaptive PID control and extreme curve handling capabilities.

## Hardware Specifications

### Sensors
- **Type**: 3× TCRT5000 Analog IR Reflective Sensors
- **Pins**: GPIO 34 (Left), GPIO 35 (Mid), GPIO 32 (Right)
- **Resolution**: 12-bit ADC (0-4095)
- **Positioning**: Weighted array for smooth curve detection

### Motors
- **Type**: N20 DC Gearmotor, 1000 RPM
- **Driver**: L298N Dual H-Bridge
- **PWM**: 500Hz frequency, 8-bit resolution (0-255)
- **Control**: LEDC peripheral channels

### Pin Configuration
```
Motor A (Left):  ENA=33, INA=25, INB=26
Motor B (Right): ENB=27, INC=14, IND=12
Calibration:     BOOT_BTN=0
```

## Control System Architecture

### 100Hz Control Loop
- **Cycle Time**: 10ms fixed period
- **Processing**: Non-blocking with timing compensation
- **Stability**: Ensures consistent PID response

### Adaptive PID Controller

#### Standard Mode (Gentle Curves)
```
Kp = 45.0
Ki = 0.15
Kd = 18.0
Base Speed = 200 (78% duty cycle)
```

#### Sharp Curve Mode (|error| > 1.2)
```
Kp = 90.0  (2.0× multiplier)
Kd = 9.0   (0.5× multiplier)
Speed Range: 20-245
```

#### Extreme Curve Mode (|error| > 2.0)
```
Pivot Turn: Inner wheel @ 20, Outer wheel @ 245
Use Case: 90° turns, U-turns, hairpin curves
```

### Error Calculation
**Weighted Position Method**:
```
weightedPosition = (L×(-1000) + M×0 + R×1000) / totalWeight
error = weightedPosition / 400.0
Range: -2.5 to +2.5
```
- Negative error → Line left, turn left
- Positive error → Line right, turn right

### Integral Wind-up Protection
- **Limit**: ±120
- **Decay**: 95% on direction change
- **Purpose**: Prevents overshooting on sharp transitions

## Sensor Processing

### Exponential Moving Average Filter
```
α = 0.8 (SENSOR_ALPHA)
filtered[n] = α × raw[n] + (1-α) × filtered[n-1]
```
**Benefits**: Reduces electrical noise while maintaining responsiveness

### Calibration System
- **Duration**: 30s standard, 10s quick mode (BOOT pressed)
- **Method**: Min-max normalization (0-1000 scale)
- **Validation**: Warns if contrast < 500 units
- **Visual Feedback**: Real-time countdown and statistics

### Line Loss Recovery
- **Timeout**: 500ms
- **Strategy**: Continue last known direction with 4.0× error
- **Memory**: Tracks last valid turn direction

## Speed Profiles

| Condition | Left Speed | Right Speed | Description |
|-----------|------------|-------------|-------------|
| Straight | 200 | 200 | Balanced tracking |
| Gentle Left | 140-200 | 200-245 | PID correction |
| Sharp Left | 20-200 | 245 | Differential speed |
| Extreme Left | 20 | 245 | Pivot turn |
| Gentle Right | 200-245 | 140-200 | PID correction |
| Sharp Right | 245 | 20-200 | Differential speed |
| Extreme Right | 245 | 20 | Pivot turn |

## Performance Characteristics

### Response Time
- **Sensor Read**: ~300μs per channel
- **PID Calculation**: ~50μs
- **PWM Update**: Instant (hardware peripheral)
- **Total Loop**: <1ms typical, 10ms guaranteed

### Curve Handling
- **Smooth Curves**: Weighted sensor positioning
- **Sharp Curves**: 2× proportional gain boost
- **90° Turns**: Automatic pivot mode
- **U-Turns**: 12:1 speed differential

## Tuning Guide

### Oscillation Issues
- **High-frequency wobble**: Reduce Kp (try 35-40)
- **Overshooting**: Increase Kd (try 22-25)
- **Slow response**: Reduce Kd (try 12-15)

### Speed Issues
- **Too cautious**: Increase baseSpeed to 220-230
- **Missing sharp turns**: Reduce sharpTurnSpeed
- **Slipping on extremes**: Increase extremeInnerSpeed to 30-40

### Sensor Issues
- **Erratic behavior**: Reduce SENSOR_ALPHA to 0.7
- **Sluggish response**: Increase SENSOR_ALPHA to 0.85
- **Line loss**: Increase LINE_LOST_TIMEOUT to 800ms

## Operational Modes

### Startup Sequence
1. Power on → Serial initialization
2. Check BOOT button state
3. Auto-calibration (30s or 10s)
4. Sensor validation
5. 2s safety delay → Start

### Calibration Process
```
1. Place robot over white surface
2. Slowly move over black line
3. Cover all sensor positions
4. System auto-detects min/max values
5. Validates contrast ratio
```

### Debug Output (100ms interval)
```
Err: -0.45 | PID: 23.5 | L:177 R:223
Err: 1.82 | PID: 85.2 | L:245 R:115 [SHARP]
Err: -2.31 | PID: -- | L:20 R:245 [EXTREME]
```

## Code Optimization Features

1. **Fast Sensor Access**: Direct GPIO read, filtered in software
2. **Hardware PWM**: LEDC peripheral for jitter-free motor control
3. **Memory Efficiency**: No dynamic allocation, fixed arrays
4. **Timing Precision**: Microsecond-level loop compensation
5. **Predictive Control**: Integral decay prevents overshoot

## Troubleshooting

### Robot Veers Off Course
- Check sensor calibration contrast (should be >500)
- Verify motor directions (swap INA/INB if reversed)
- Adjust baseSpeed if too fast for track

### Slow Response to Curves
- Increase Kp for sharper correction
- Reduce SENSOR_ALPHA for faster updates
- Lower extremeInnerSpeed for tighter pivots

### Oscillates on Straight Line
- Reduce Kp (overreacting)
- Increase Kd (damping)
- Check for mechanical play in motors

### Fails Extreme Curves
- Verify isExtremeCurve() threshold (try 1.8)
- Increase outer wheel maxSpeed
- Reduce extremeInnerSpeed for more pivot

## Advanced Modifications

### Track-Specific Tuning
```cpp
// High-speed track
baseSpeed = 230;
SHARP_CURVE_KP_MULT = 1.5;

// Technical track (many curves)
baseSpeed = 180;
SHARP_CURVE_KP_MULT = 2.5;
extremeInnerSpeed = 15;
```

### Adding Acceleration Ramps
```cpp
void setMotorSpeedSmooth(int target_L, int target_R) {
  static int current_L = 0, current_R = 0;
  current_L += constrain(target_L - current_L, -5, 5);
  current_R += constrain(target_R - current_R, -5, 5);
  setMotorSpeed(current_L, current_R);
}
```

## System Requirements
- **ESP32 DevKit**: Any variant with 3× ADC channels
- **Power**: 7.4V LiPo (2S) or 6× AA batteries
- **Current**: 2A peak per motor
- **Track**: 15-25mm black line on white surface

## License & Credits
Optimized implementation for competitive line following robots. Designed for reliability, speed, and extreme maneuverability.

---
**Version**: 1.0  
**Target Platform**: ESP32 (Arduino Framework)  
**Optimization Level**: Competition-grade
