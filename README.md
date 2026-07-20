# Professional-Grade 4-DOF Robotic Arm Controller

A highly optimized, production-ready firmware for Arduino Uno (ATmega328P) controlling a 4-DOF robotic arm with advanced motion profiling, inverse kinematics, and robust serial communication.

## Features

### Core Engineering Principles
- **Non-Blocking Architecture**: State-machine design using `millis()` for highly responsive operation
- **Trapezoidal Motion Profiling**: Smooth acceleration/deceleration curves to prevent mechanical stress
- **Geometric Inverse Kinematics**: Mathematical solution for 3-DOF positioning (Base/Shoulder/Elbow)
- **Workspace Validation**: Comprehensive safety constraints to prevent self-collision and overreach
- **EEPROM Persistence**: Calibration data survives power cycles with validation
- **Robust Serial Protocol**: Command parsing with error handling and validation

### Motion Control
- **Trapezoidal Velocity Profiles**: Three-phase motion (acceleration → constant velocity → deceleration)
- **Configurable Constraints**: Per-joint velocity and acceleration limits
- **Emergency Stop**: Immediate halt capability for safety
- **Smooth Gripper Control**: Percentage-based open/close with motion profiling

### Kinematics
- **Inverse Kinematics**: Maps XYZ coordinates (mm) to joint angles (degrees)
- **Forward Kinematics**: Computes end-effector position from joint angles (for debugging)
- **Geometric Solution**: Law of cosines for efficient computation
- **Workspace Checking**: Validates reachability before motion

## Hardware Requirements

### Microcontroller
- **Arduino Uno** (ATmega328P)
- **2KB SRAM**, **32KB Flash**, **1KB EEPROM**

### Servo Motors (4x PWM)
- **Base Servo**: Pin 3 - Rotation in X-Y plane
- **Shoulder Servo**: Pin 5 - Elevation in vertical plane  
- **Elbow Servo**: Pin 6 - Reach extension
- **Gripper Servo**: Pin 9 - Open/close actuation

### Mechanical Specifications
- **Link Lengths** (adjustable in `Arm.h`):
  - Shoulder link: 100mm (base to shoulder joint)
  - Elbow link: 100mm (shoulder to elbow joint)
  - Hand offset: 50mm (elbow to gripper)

### Communication
- **Serial**: 115200 baud rate
- **Format**: ASCII commands with newline termination

## Software Architecture

### Memory Optimization (ATmega328P Constraints)
- **Float vs Double**: Uses 32-bit floats to save flash/SRAM
- **Fixed-Size Buffers**: Prevents heap fragmentation
- **Efficient Structures**: Joint structure ~53 bytes × 4 = ~212 bytes total
- **EEPROM Wear Leveling**: Strategic address usage for longevity

### State Machine Design
```
IDLE → ACCEL → CONST_VEL → DECEL → IDLE
```

### File Structure
```
Robotic Arm/
├── include/
│   └── Arm.h          # Class definitions and constants
├── src/
│   ├── Arm.cpp        # Implementation of kinematics and motion control
│   └── main.cpp       # Serial parser and main loop
├── platformio.ini     # PlatformIO configuration
└── README.md          # This file
```

## Kinematics Mathematics

### Inverse Kinematics Solution

The arm uses a geometric approach with the following mathematical derivation:

1. **Base Angle (Azimuth)**:
   ```
   θ_base = atan2(y, x)
   ```

2. **Radial Projection**:
   ```
   r = sqrt(x² + y²)
   ```

3. **Elbow Angle** (Law of Cosines):
   ```
   cos(θ_elbow) = (r² + z² - L₁² - L₂²) / (2·L₁·L₂)
   θ_elbow = acos(cos(θ_elbow))
   ```

4. **Shoulder Angle**:
   ```
   k₁ = L₁ + L₂·cos(θ_elbow)
   k₂ = L₂·sin(θ_elbow)
   θ_shoulder = atan2(z, r) - atan2(k₂, k₁)
   ```

Where:
- `L₁` = LINK_SHOULDER (shoulder link length)
- `L₂` = LINK_ELBOW (elbow link length)
- `(x, y, z)` = Target coordinates in millimeters

### Workspace Constraints
- **Height**: -50mm to +200mm (Z-axis)
- **Reach**: 20mm to 180mm radial distance
- **Maximum Reach**: LINK_SHOULDER + LINK_ELBOW
- **Minimum Reach**: |LINK_SHOULDER - LINK_ELBOW|

## Motion Profiling

### Trapezoidal Velocity Profile

The firmware implements a three-phase motion profile:

1. **Acceleration Phase**:
   ```
   position = 0.5 · a · t²
   ```
   - Smooth ease-in from rest
   - Duration depends on target velocity and acceleration limits

2. **Constant Velocity Phase**:
   ```
   position = position_accel + v · t
   ```
   - Maintains peak velocity for efficiency
   - Skipped for short movements (triangle profile)

3. **Deceleration Phase**:
   ```
   position = total_distance - 0.5 · a · (t_total - t)²
   ```
   - Smooth ease-out to target
   - Mirrors acceleration for symmetry

### Default Constraints
- **Maximum Velocity**: 120°/s (conservative for safety)
- **Maximum Acceleration**: 300°/s²
- **Minimum Profile Duration**: 20ms (prevents division by zero)

## Serial Command Protocol

### Command Format
All commands are ASCII strings terminated by newline (`\n`).

### Available Commands

#### Motion Commands
```
G0 X[x] Y[y] Z[z] F[f]  - Move to XYZ coordinates (mm)
                         X, Y, Z: Target position (required)
                         F: Feed rate in mm/s (optional, default 50)
```

**Example**: `G0 X100 Y50 Z120 F75`

#### Calibration Commands
```
HOME                   - Move to calibrated home position
SAVE                   - Save current position as home to EEPROM
LOAD                   - Load home position from EEPROM
RESET                  - Factory reset EEPROM (clears calibration)
```

#### Control Commands
```
STOP                   - Emergency stop (immediate halt)
GRIP [0-100]          - Set gripper (0=closed, 100=open)
STATUS                 - Show current joint angles and position
HELP                   - Display available commands
```

### Response Format
- **Success**: `OK <message>`
- **Error**: `ERR <error description>`

### Example Session
```
> HELP
=== AVAILABLE COMMANDS ===
G0 X[x] Y[y] Z[z] F[f] - Move to coordinates (mm)
HOME                   - Move to home position
SAVE                   - Save current as home
...

> G0 X100 Y50 Z120 F50
OK Motion started

> STATUS
=== ARM STATUS ===
Joints: B=45.0 S=90.0 E=30.0 G=90.0
Position: X=100.0 Y=50.0 Z=120.0
Moving: YES
Calibration: VALID
==================

> STOP
OK Emergency stop activated
```

## EEPROM Memory Management

### Memory Map
```
Address 0-1:   Base home position (int16_t)
Address 2-3:   Shoulder home position (int16_t)
Address 4-5:   Elbow home position (int16_t)
Address 6-7:   Gripper home position (int16_t)
Address 8-9:   Validation magic number (0xA55A)
```

### Calibration Procedure
1. Manually position arm to desired home position
2. Send `SAVE` command to store to EEPROM
3. Home position persists across power cycles
4. Use `RESET` to clear and return to factory defaults

### Validation
- Magic number (0xA55A) validates EEPROM integrity
- Invalid data defaults to factory settings (90° for all joints)
- Range checking prevents loading corrupted values

## Installation and Setup

### Prerequisites
- [PlatformIO](https://platformio.org/) extension for VS Code
- Arduino Uno board
- 4x PWM servo motors
- External 5V power supply recommended for servos

### Build Instructions
1. Clone or download this project
2. Open in VS Code with PlatformIO
3. Connect Arduino Uno via USB
4. Press PlatformIO "Upload" button
5. Open Serial Monitor at 115200 baud

### Hardware Connection
```
Arduino Uno          Servo Motors
-----------          ------------
Pin 3    -------->   Base Servo Signal
Pin 5    -------->   Shoulder Servo Signal  
Pin 6    -------->   Elbow Servo Signal
Pin 9    -------->   Gripper Servo Signal
5V       -------->   Servo Power (external supply recommended)
GND      -------->   Servo Ground
```

## Configuration

### Adjusting Link Lengths
Edit `include/Arm.h`:
```cpp
#define LINK_SHOULDER 100.0f  // Your shoulder link length (mm)
#define LINK_ELBOW    100.0f  // Your elbow link length (mm)
#define LINK_HAND     50.0f   // Your hand offset (mm)
```

### Adjusting Workspace Constraints
```cpp
#define MIN_REACH     20.0f   // Minimum radial distance (mm)
#define MAX_REACH     180.0f  // Maximum radial distance (mm)
#define MIN_Z         -50.0f  // Minimum height (mm)
#define MAX_Z         200.0f  // Maximum height (mm)
```

### Adjusting Motion Constraints
```cpp
#define DEFAULT_MAX_VELOCITY     120.0f  // deg/s
#define DEFAULT_MAX_ACCELERATION 300.0f  // deg/s²
```

### Changing Servo Pins
```cpp
#define PIN_BASE     3   // Base servo PWM pin
#define PIN_SHOULDER 5   // Shoulder servo PWM pin
#define PIN_ELBOW    6   // Elbow servo PWM pin
#define PIN_GRIPPER  9   // Gripper servo PWM pin
```

## Usage Examples

### Basic Motion Sequence
```
G0 X100 Y0 Z100 F50    # Move to position
G0 X100 Y50 Z100       # Rotate base
G0 X100 Y50 Z50        # Lower arm
HOME                   # Return to home
```

### Pick and Place Pattern
```
G0 X150 Y0 Z80 F75     # Move above pick location
G0 X150 Y0 Z30 F30     # Lower to pick
GRIP 0                 # Close gripper
G0 X150 Y0 Z80 F75     # Lift
G0 X50 Y90 Z80 F75     # Move above place location
G0 X50 Y90 Z30 F30     # Lower to place
GRIP 100               # Open gripper
G0 X50 Y90 Z80 F75     # Lift
HOME                   # Return home
```

### Calibration Workflow
```
# Manually position arm to desired home
SAVE                   # Save current position
# Power cycle arm
LOAD                   # Verify calibration loaded
STATUS                 # Check current state
```

## Troubleshooting

### Arm Not Moving
- Check servo connections and power supply
- Verify Serial Monitor is set to 115200 baud
- Send `STATUS` command to check joint angles
- Ensure target position is within workspace

### "Target Unreachable" Error
- Verify XYZ coordinates are within workspace
- Check link length configuration matches your hardware
- Use `STATUS` to see current position for reference

### Erratic Servo Behavior
- Ensure adequate power supply (external 5V recommended)
- Check for loose connections
- Verify servo signal wires are not noisy
- Reduce maximum velocity/acceleration if needed

### EEPROM Issues
- Use `RESET` to clear corrupted calibration
- Check EEPROM write endurance (100,000 cycles typical)
- Verify magic number validation is working

### Memory Constraints
- Monitor SRAM usage if adding features
- Use `float` instead of `double` for calculations
- Avoid dynamic memory allocation
- Keep serial buffers minimal

## Performance Characteristics

### Timing
- **Loop Frequency**: ~50Hz (20ms cycle time)
- **Servo Update Rate**: 50Hz
- **Serial Latency**: <1ms for command processing

### Memory Usage
- **SRAM**: ~400 bytes (Arm class + buffers)
- **Flash**: ~12KB (compiled firmware)
- **EEPROM**: 10 bytes (calibration data)

### Motion Accuracy
- **Position Resolution**: 1° (servo limitation)
- **Repeatability**: ±2° (typical servo hysteresis)
- **Workspace Coverage**: ~90% of theoretical reach

## Safety Considerations

### Mechanical Safety
- Always test motions at low speeds first
- Keep hands clear of moving parts
- Use current-limiting power supply
- Install emergency stop button if possible

### Software Safety
- Workspace validation prevents overreach
- Emergency stop command halts all motion
- EEPROM validation prevents corrupted calibration
- Motion profiling reduces mechanical stress

### Electrical Safety
- Use external power supply for servos
- Common ground between Arduino and servo supply
- Include fuse or current limiter
- Check for short circuits before powering

## Future Enhancements

Potential improvements for advanced users:
- Add trajectory interpolation for curved paths
- Implement PID control for precise positioning
- Add encoder feedback for closed-loop control
- Support for coordinated multi-arm systems
- G-code file parsing for complex sequences
- Wireless control via Bluetooth/WiFi

## License

This firmware is provided as-is for educational and professional use.

## Contributing

Contributions are welcome! Please ensure:
- Code follows existing style and documentation
- Features maintain memory constraints
- All changes are tested on hardware
- Documentation is updated accordingly

## Version History

- **v1.0** - Initial professional-grade release
  - Trapezoidal motion profiling
  - Geometric inverse kinematics
  - EEPROM calibration persistence
  - Robust serial command parser
  - Comprehensive workspace validation
