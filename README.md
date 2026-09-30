# INF2004 Embedded Systems Project — Group 18

## Autonomous Robotic Car Challenge

An autonomous robotic car that navigates a predefined course entirely under its own
control — no remote control — using onboard sensors and embedded software to sense,
decide, move and report as one integrated system.

## Overview

The car must complete a course consisting of:

- A line-following track
- Directional barcodes placed at selected locations
- Speed humps of varying heights
- Static obstacles placed along the track

While navigating, it collects environmental information and makes autonomous decisions
based on sensor input. The project emphasises embedded systems design, sensor
integration, resource-efficient programming, real-time decision making, autonomous
navigation, and hardware-software co-design.

## Platform & Constraints

- **Board:** Raspberry Pi Pico + Robo Pico
- **RTOS:** micro T-Kernel
- **Language:** C
- **SDK:** Pico C SDK
- **Coding standard:** Barr C Coding Standard
- Must follow the template GitHub repo structure

As embedded systems are resource-constrained, the software must make prudent use of
processing, memory, communication, and power resources.

## Mission Requirements

The robotic car shall:

1. Follow a line track autonomously.
2. Detect and decode barcodes positioned along the track.
3. Execute navigation commands encoded by barcodes (Turn Left, Turn Right, Go Straight,
   U-Turn).
4. Detect humps encountered along the route.
5. Measure and report the highest hump peak experienced during the run.
6. Detect obstacles located on or near the line.
7. Profile obstacle shape and position using ultrasonic sensing.
8. Navigate around obstacles while avoiding collisions.
9. Reacquire the original line after bypassing an obstacle.
10. Report telemetry data through WiFi communication.

### Example barcode commands

| Barcode | Command     |
| ------- | ----------- |
| A       | Turn Left   |
| B       | Turn Right  |
| C       | Go Straight |
| D       | U-Turn      |

## System Architecture

**Sensor layer**
- IR sensors (line + barcode)
- Wheel encoders
- IMU
- Ultrasonic sensor + servo
- Barcode sensor

**Control layer**
- Motion control
- Line following
- Obstacle avoidance
- Navigation logic

**Communication layer**
- WiFi
- MQTT
- Telemetry

A central **Vehicle Controller** owns mission state and coordination, and routes data
and commands between the five subsystems below.

## Team Structure

Each member ("buddy") owns a subsystem, but the robot is assessed as one integrated
system — everyone participates in integration, system testing, debugging, and the
final demonstration, and understands the other subsystems' interfaces.

### Buddy 1 — WiFi Communication, Command & Telemetry (Fiona)

- **Hardware:** WiFi
- **Tasks:** WiFi connectivity, MQTT communication, topic design, telemetry message
  structures, telemetry publishing, command subscription, connection recovery,
  heartbeat/status reporting.
- **Deliverables:** Communication API, telemetry framework, MQTT documentation,
  telemetry demonstration.

### Buddy 2 — Motion Control System (Ryan)

- **Hardware:** Motors + wheel encoders
- **Tasks:** Motor driver and encoder integration, speed/distance estimation, PID speed
  control, straight-line correction, encoder-based turning, motion calibration.
- **Required APIs:** `moveForward(distance)`, `moveBackward(distance)`,
  `turnLeft(angle)`, `turnRight(angle)`, `stop()`
- **Deliverables:** Motion-control library, PID tuning report, motion accuracy
  evaluation.

### Buddy 3 — Barcode Decoding & IR Line Following (Zane)

- **Hardware:** 3× IR sensors (2 for line detection, 1 for barcode reading)
- **Tasks:** Sensor calibration, line position estimation, line-following algorithm,
  junction detection, barcode detection/decoding, navigation command generation.
- **Deliverables:** Line-following module, barcode decoder, navigation command
  interface.

### Buddy 4 — IMU-Based Motion & Terrain Monitoring (Jerald)

- **Hardware:** IMU
- **Tasks:** IMU integration and calibration, tilt/hump detection, peak hump height
  measurement, motion classification, collision detection, turn-rate monitoring, motion
  event reporting.
- **Deliverables:** IMU processing module, hump-detection algorithm, terrain analysis
  report.

### Buddy 5 — Adaptive Ultrasonic Scanning & Obstacle Profiling (Max)

- **Hardware:** Ultrasonic sensor + servo
- **Tasks:** Coarse scan then fine scan around obstacles, obstacle location/width/
  clearance estimation, avoidance planning (stop/turn/reverse), line search and
  reacquisition after bypass.
- **Deliverables:** Scanning subsystem, obstacle profile generator, avoidance and
  recovery algorithms.

## Assessment Criteria

- Successful completion of mission objectives
- Navigation accuracy
- Obstacle avoidance effectiveness
- Barcode recognition reliability
- Hump measurement capability
- System robustness
- Software design quality
- Resource efficiency
- Team integration quality
- Demonstration performance

## Development Approach

1. Define inputs, outputs and ownership for each subsystem interface.
2. Create repeatable subsystem tests.
3. Record measurements and calibration data.
4. Integrate early through stable APIs.
5. Practise the Week 10 demonstration.

## Getting Started

<!-- TODO: fill in once the template repo/toolchain is set up -->

### Prerequisites

- Raspberry Pi Pico SDK toolchain (CMake, arm-none-eabi-gcc)
- micro T-Kernel RTOS sources
- Robo Pico board

### Build

<!-- TODO: build instructions -->

### Flash / Run

<!-- TODO: how to flash/run on hardware -->

## Repository Structure

<!-- TODO: describe folders once code is added -->
