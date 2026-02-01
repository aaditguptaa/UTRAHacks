# UTRA Hacks Copilot Instructions

## Project Overview
This is an Arduino-based obstacle detection and avoidance robot combining:
- **Sensor module**: HC-SR04 ultrasonic sensor for distance detection
- **Motor control**: Two DC motors (pins 2, 11 for left; 12, 13 for right)
- **Servo mechanism**: Servo motor on pin A4 for scanning obstacles

## Architecture & Data Flow

### Main Control Loop (`ultrasonic_sensor.ino`)
1. **Distance sensing**: Ultrasonic sensor triggers 10µs pulse, measures echo time
2. **Distance calculation**: `distance = duration * 0.034 / 2` (cm units)
3. **Decision logic**:
   - `distance > 15 cm`: Move forward
   - `distance < 10 cm`: Execute obstacle response sequence

### Obstacle Response Sequence
When obstacle detected:
1. Stop both motors
2. Scan with servo (180°, 90°)
3. Reverse (500ms)
4. Turn left (500ms)
5. Move forward, then correct to straight
6. Resume forward movement

## Motor Control Conventions

### Pin Mapping & Logic
```cpp
Left Motor:  pin1=2 (pin A), pin2=11 (pin B)
Right Motor: pin1=12 (pin A), pin2=13 (pin B)

// Motor direction truth table (A, B):
LOW,  LOW   → STOP/BRAKE
LOW,  HIGH  → BACKWARD
HIGH, LOW   → FORWARD
HIGH, HIGH  → (unused, avoid)
```

**Pattern**: For synchronized movement, apply same state to both motors' corresponding pins.

## Key Implementation Patterns

### Timing & Delays
- Ultrasonic sensor: Requires 2µs low + 10µs pulse minimum
- Motor transitions: Use 500ms base delay (marked "time needs test" - may need calibration)
- Serial debug: 115200 baud rate

### Pin Definitions
All pins must be declared as `const int` at sketch top. Current issues:
- `trigPin` and `echoPin` lack numeric values
- Servo attach uses `A4` (analog pin, valid for Arduino)

## Common Issues & Fixes

### Known Bugs in Current Code
- Line 23: `Serail.begin()` → should be `Serial.begin()`
- Line 31: Missing semicolon after `digitalWrite`
- Line 47: `Serial.prrint()` → should be `Serial.print()`
- Line 62: Missing semicolon after `digitalWrite(mtr1pin LOW)`
- Line 85: Missing semicolon after `delay(500)`
- Line 90: Mismatched parenthesis in `digitalWrite(mtr2pin, HIGH;`
- Servo initialization: Called `Myservo` in loop but declared as `myservo`

### Servo Reference Issues
- `servo_arm` file declares `myservo` but main code references `Myservo` (case-sensitive)
- Ensure consistent naming: use lowercase `myservo` throughout

## When Modifying Code
- Validate all pin definitions have numeric values
- Test timing delays on actual hardware (500ms values are estimates)
- Check serial output capitalization (`Serial`, not `Serail`)
- Maintain motor symmetry: parallel logic for left/right pairs
- Remember servo pulses need servo library and proper attachment

## Testing Workflow
1. Verify sensor pins are triggered correctly (test with Serial output)
2. Validate distance calculation matches physical measurements
3. Test motor direction logic with manual pin states
4. Calibrate obstacle response delays based on robot weight/speed
5. Check servo scanning doesn't interfere with motor timing
