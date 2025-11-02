Autonomous line follower robot built using ESP32, N20 motors, L298N driver, and TCRT5000 sensors.

# Overview

## 🧩 Hardware

-   ESP32 DevKit
-   N20 Gear Motors
-   L298N Motor Driver
-   5× TCRT5000 Sensor Array
-   3× 18650 Battery Pack (11.1V)
-   PVC Chassis 9.45×7.87 in

## 🧠 Features``

-   PID-based line following
-   Calibration routine for sensors
-   Adjustable motor PWM control
-   Modular code structure for testing

## ⚙️ Folder Overview

| Folder        | Purpose                   |
| ------------- | ------------------------- |
| `code/`       | Firmware and test code    |
| `circuits/`   | Circuit schematics        |
| `mechanical/` | Chassis and CAD designs   |
| `docs/`       | Documentation and rules   |
| `data/`       | Calibration and log files |

# 🚗 Core Idea

The Line Follower Robot (LFR) autonomously follows a black line on a white track using sensors and control algorithms. It must handle **curves, intersections, gaps, and checkpoints** without human input during operation.

# ⚙️ Major Subsystems

## 1. Mechanical Subsystem (Chassis & Motors)

-   Lightweight and balanced chassis (≤ 25×25×18 cm, ≤ 1.2 kg).
-   Two N20 gear motors with differential drive.
-   One free or caster wheel for balance.
-   Goal: stability, low friction, easy maintenance.

## 2. Electrical Subsystem (Power & Circuitry)

-   18650 battery pack with BMS and kill switch.
-   Voltage < 12V; regulated 3.3V for ESP32.
-   Motor driver (preferably TB6612FNG or DRV8833). Using L298N
-   Includes capacitors for noise suppression and safe wiring layout.

## 3. Sensing Subsystem (IR Line Sensor Array)

-   TCRT5000 IR sensors arranged as an array (usually 5).
-   Detect reflected light: black = high value, white = low value.
-   Calibrated to adapt to ambient light and surface conditions.

## 4. Control Subsystem (ESP32 Software)

-   Reads sensor values and computes line position error.
-   Uses [[PID Control]] to adjust motor speeds.
-   Implements recovery logic for gaps, checkpoints, and intersections.
-   Operates autonomously during runs.

# 🧠 How the Robot Completes a Run

## 1. Calibration Phase

-   Measures black vs white reflectivity.
-   Determines sensor thresholds and normalization parameters.
-   Compensates for lighting and track surface.

## 2. Start Phase

-   Robot starts from a 30×30 cm black zone.
-   After calibration, robot runs without human input.

## 3. Control Loop

-   Reads sensor data every 10–20 ms.
-   Calculates line position error using weighted averages.
-   Applies PID algorithm to adjust motor PWMs:
    -   left = base_speed - steering
    -   right = base_speed + steering
-   Keeps the robot centered on the black line.

## 4. Handling Obstacles

-   **Curves**: PID smoothly adjusts steering.
-   **Sharp turns**: higher D-term or lower speed prevents overshoot.
-   **Gaps (15 cm)**: robot coasts briefly, then reacquires line.
-   **Checkpoints**: recognized by wide black areas; progress logged.

## 5. Finish & Scoring

-   Robot stops automatically at the endpoint or last checkpoint.
-   Points awarded based on checkpoints crossed and completion time.

# ⚡ Control System Summary

## Control Flow

1. Read IR sensor values.
2. Compute line error (weighted average method).
3. Use PID to calculate steering correction.
4. Apply motor PWMs for left/right wheels.
5. Detect and handle checkpoints or line loss.

## PID Overview

-   **P**: Corrects based on present error (fast response).
-   **I**: Corrects steady drift over time (removes bias).
-   **D**: Damps oscillations by reacting to rate of change.

## Tuning Tips

1. Start with I = 0, D = 0.
2. Increase P until oscillation begins, then reduce slightly.
3. Add D to smooth motion.
4. Add small I if robot drifts off line.

# ⚙️ Track Handling Logic

## Line Loss (Gaps)

-   Continue briefly in the last known direction (100–250 ms).
-   If line not found, perform slow scanning turn.
-   Use encoders or timing for controlled recovery.

## Checkpoints

-   Detected by wide black areas under multiple sensors.
-   Save progress to `last_checkpoint`.
-   On restart, resume from last checkpoint.

# 🧩 Integration Notes

-   Keep sensor cables short; ensure common ground.
-   Add capacitors to stabilize voltage during motor spikes.
-   Disable Wi-Fi/Bluetooth before inspection.
-   Verify kill switch operation.

# 🧪 Testing Plan

1. **Unit Testing:** Sensors, motors, kill switch individually.
2. **Integration Testing:** Basic line following and turning.
3. **Feature Testing:** Gaps, intersections, and checkpoints.
4. **Performance Testing:** Timed full runs and recovery tests.
5. **Pre-Competition Check:** Verify all rules and dimensions.

# 🧭 LFR PVC Chassis Dimensions (in Inches)

## ⚙️ Competition Limits

-   **Max Width:** 9.84 in
-   **Max Length:** 9.84 in
-   **Max Height:** 7.09 in

## 🧱 Recommended Base Plate

-   **Chassis plate (main base):** 9.45 × 7.87 × 0.14 in
-   **PVC thickness:** 0.12–0.16 in
    > Keep total robot size under **9.45 × 7.87 × 7.09 in** for safety margin and compact handling.

## 📐 Core Layout

| Feature                                 | Measurement (inches) |
| --------------------------------------- | -------------------- |
| Wheel spacing (center-to-center)        | 5.51                 |
| Wheelbase (front–rear axle)             | 3.94                 |
| Caster offset (behind rear axle)        | 0.59–0.98            |
| Sensor bar offset (ahead of front axle) | 0.39–0.71            |
| Sensor-to-ground clearance              | 0.08–0.16            |
| Battery area                            | 2.76 × 1.57          |
| ESP32 area                              | 1.97 × 1.18          |

## 🔩 Mounting Holes

| Mount                       | Hole Spacing (inches) |
| --------------------------- | --------------------- |
| Motor bracket holes         | 0.87 apart            |
| ESP32 standoffs rectangle   | 1.89 × 0.98           |
| Sensor bar adjustment slots | ±0.20–0.39            |

## 📏 Component Heights

| Component                | Height (inches) |
| ------------------------ | --------------- |
| Battery pack             | ≤ 1.38          |
| ESP32 board + connectors | ≤ 0.59          |
| Sensor board clearance   | 0.08–0.16       |
| Total robot height       | ≤ 7.09          |

## 🛞 Motor & Wheel Specs

| Parameter                      | Measurement (inches) |
| ------------------------------ | -------------------- |
| Wheel diameter                 | 1.18–1.57            |
| Wheel offset from chassis edge | 0.24–0.31            |
| Wheelbase (axle–axle)          | 3.94                 |

## ⚖️ Center of Gravity Notes

-   Keep **battery and ESP32** centered along the width.
-   Place the battery **slightly forward (~0.4–0.8 in)** of the motor axis.
-   Aim for a **balanced or slightly front-heavy** weight distribution (≈45% front, 55% rear).

### Template Coordinates (Inches)

| Component             | X (left–right) | Y (front–rear)  |
| --------------------- | -------------- | --------------- |
| Left motor center     | 1.97 from left | 3.94 from rear  |
| Right motor center    | 7.48 from left | 3.94 from rear  |
| Sensor bar centerline | —              | 0.59 from front |
| Battery center        | 4.72 from left | 2.36 from rear  |

## 🧰 Construction Notes

-   Use **⅛-inch (3 mm)** PVC or **3/16-inch (4.5 mm)** if heavier.
-   **Deburr edges** and **round corners (⅛–¼ inch radius)** to protect the track.
-   Keep bolts **flush or recessed** to avoid scraping.
-   Add **cross-bracing** or 0.08–0.12 in PVC strips underneath if the plate flexes.
-   Use **foam tape or Velcro** for lightweight mounting of ESP32 and battery.

## 🎯 How to Calibrate

1. Upload the sketch.
2. Open Serial Monitor (115200 baud).
3. Move each sensor across the black line and white surface.
    - Note readings.
    - Set threshold between average black and white.
4. Start with:

```
Kp = 25, Ki = 0, Kd = 8
```

5. Tune gradually:
    - If it oscillates → reduce Kp or increase Kd
    - If it’s slow to correct → increase Kp
    - If it drifts → increase Ki slightly (rarely needed)

## 🔌 Cable Management

-   Route motor wires along side edges and secure with zip-ties.
-   Keep sensor wires short (< 3 in).
-   Use **heatshrink tubing** for all exposed solder joints.
-   Add a **removable connector** for battery input.

## 🧨 Kill Switch

-   Mount main **kill switch** on the top or side, clearly visible.
-   Use recessed mounting so it won’t snag.
-   Label auxiliary switches if used.

## ✅ Final Checklist

-   [ ] Base plate: 9.45 × 7.87 in
-   [ ] Wheel spacing: 5.5 in center-to-center
-   [ ] Wheelbase: 3.9–4.0 in
-   [ ] Sensor bar: 0.6 in forward of front axle
-   [ ] Sensor clearance: 0.1 in from surface
-   [ ] Battery centered, ≤ 1.4 in high
-   [ ] ESP32 board ≤ 0.6 in high
-   [ ] Kill switch reachable from top
-   [ ] Wiring tidy and insulated
