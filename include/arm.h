// Arm.h - Professional-Grade 4-DOF Robotic Arm Controller
// Target: Arduino Uno (ATmega328P) - 2KB SRAM, 32KB Flash
//
// FEATURES:
// - Non-blocking state machine architecture using millis()
// - Trapezoidal motion profiling with acceleration/deceleration phases
// - Geometric inverse kinematics for 3-DOF positioning (Base/Shoulder/Elbow)
// - Workspace validation with safety constraints
// - EEPROM persistence for calibration/home positions
// - Robust serial command parsing with error handling
// - Memory-optimized for ATmega328P constraints
//
// KINEMATICS:
// - Base: Rotation in X-Y plane (azimuth)
// - Shoulder: Elevation angle in vertical plane
// - Elbow: Relative angle for reach extension
// - Gripper: Open/close percentage control
//
// MOTION PROFILING:
// - Implements trapezoidal velocity profile:
//   1. Acceleration phase (ease-in)
//   2. Constant velocity phase
//   3. Deceleration phase (ease-out)
// - Prevents mechanical stress and smooth movements
//
// MEMORY MANAGEMENT:
// - Uses float instead of double to save flash/sram
// - Fixed-size buffers to prevent heap fragmentation
// - EEPROM wear leveling through strategic address usage

#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Servo.h>

// ============================================================
// PHYSICAL CONSTANTS - Calibrate for your specific arm
// ============================================================

// Link lengths in millimeters (measure your arm precisely)
#define LINK_SHOULDER 100.0f  // Base to shoulder joint
#define LINK_ELBOW    100.0f  // Shoulder to elbow joint
#define LINK_HAND     50.0f   // Elbow to gripper (optional offset)

// Workspace constraints (mm) - prevents self-collision and overreach
#define MIN_REACH     20.0f   // Minimum distance from base center
#define MAX_REACH     180.0f  // Maximum reach (shoulder + elbow - margin)
#define MIN_Z         -50.0f  // Minimum height (below base plane)
#define MAX_Z         200.0f  // Maximum height

// Servo pin assignments - PWM capable pins on Uno
#define PIN_BASE     3   // Base rotation (PWM)
#define PIN_SHOULDER 5   // Shoulder elevation (PWM)
#define PIN_ELBOW    6   // Elbow extension (PWM)
#define PIN_GRIPPER  9   // Gripper actuation (PWM)

// EEPROM memory map - ATmega328P has 1024 bytes
#define EEPROM_ADDR_BASE      0   // 2 bytes
#define EEPROM_ADDR_SHOULDER  2   // 2 bytes  
#define EEPROM_ADDR_ELBOW     4   // 2 bytes
#define EEPROM_ADDR_GRIPPER   6   // 2 bytes
#define EEPROM_ADDR_MAGIC     8   // 2 bytes (validation)
#define EEPROM_MAGIC_VALUE    0xA55A  // Valid EEPROM marker

// Motion profile constants
#define DEFAULT_MAX_VELOCITY     120.0f  // deg/s (conservative for safety)
#define DEFAULT_MAX_ACCELERATION 300.0f  // deg/s^2
#define MIN_PROFILE_DURATION     20      // ms (prevents division by zero)
#define SERVO_UPDATE_RATE        20      // ms (50Hz servo refresh)

// ============================================================
// MOTION PROFILE STATES
// ============================================================
enum ProfileState {
  PROFILE_IDLE = 0,      // No motion in progress
  PROFILE_ACCEL,         // Acceleration phase
  PROFILE_CONST_VEL,     // Constant velocity phase
  PROFILE_DECEL          // Deceleration phase
};

// ============================================================
// JOINT STRUCTURE - Memory-optimized for 4 joints
// ============================================================
struct Joint {
  Servo servo;           // Servo object (8 bytes)
  uint8_t pin;           // PWM pin assignment (1 byte)
  
  // Current state
  float currentDeg;      // Current commanded position (4 bytes)
  float targetDeg;       // Target position (4 bytes)
  
  // Motion constraints
  float maxVelDegPerS;   // Maximum velocity (deg/s) (4 bytes)
  float maxAccDegPerS2;  // Maximum acceleration (deg/s²) (4 bytes)
  
  // Motion profile state
  ProfileState state;    // Current profile phase (1 byte)
  unsigned long profileStartMs;  // Profile start timestamp (4 bytes)
  unsigned long profileDurationMs; // Total duration (4 bytes)
  float startDeg;        // Starting position (4 bytes)
  float peakVel;         // Peak velocity during profile (4 bytes)
  float accelTime;       // Duration of accel phase (s) (4 bytes)
  float decelTime;       // Duration of decel phase (s) (4 bytes)
  bool moving;           // Motion in progress flag (1 byte)
  
  // Total: ~53 bytes per joint × 4 = ~212 bytes (well within 2KB SRAM)
};

// ============================================================
// MAIN ARM CLASS - Public Interface
// ============================================================
class Arm {
public:
  Arm();
  
  // Initialization - call once in setup()
  void begin();
  
  // Non-blocking periodic update - call every loop iteration
  // Handles all motion profiling and servo updates
  void update();
  
  // ============================================================
  // KINEMATICS INTERFACE
  // ============================================================
  
  // Inverse Kinematics: Maps XYZ coordinates (mm) to joint angles (deg)
  // Input:  x, y, z - Target position in millimeters
  // Output: baseDeg, shoulderDeg, elbowDeg - Computed joint angles
  // Returns: true if position is reachable, false otherwise
  // Mathematical basis Geometric solution using law of cosines
  bool inverseKinematics(float x, float y, float z, 
                        float &baseDeg, float &shoulderDeg, float &elbowDeg);
  
  // Forward Kinematics: Maps joint angles to XYZ position (useful for debugging)
  // Input: baseDeg, shoulderDeg, elbowDeg - Joint angles in degrees
  // Output: x, y, z - Computed position in millimeters
  void forwardKinematics(float baseDeg, float shoulderDeg, float elbowDeg,
                        float &x, float &y, float &z);
  
  // ============================================================
  // MOTION CONTROL INTERFACE
  // ============================================================
  
  // Move to XYZ coordinates with motion profiling (non-blocking)
  // Input: x, y, z - Target position (mm)
  //        feedRateMmPerS - Speed in mm/s (default: 50)
  // Returns: true if motion started, false if unreachable
  bool moveToXYZ(float x, float y, float z, float feedRateMmPerS = 50.0f);
  
  // Emergency stop - immediately halts all motion
  void emergencyStop();
  
  // Check if any joint is currently moving
  bool isMoving() const;
  
  // Move to saved home position (non-blocking)
  void moveToHome(float feedRateDegPerS = 30.0f);
  
  // Gripper control (0-100% open)
  // 0% = fully closed, 100% = fully open
  void setGripperPercent(uint8_t pct);
  
  // ============================================================
  // CALIBRATION & PERSISTENCE
  // ============================================================
  
  // Save current joint positions as home to EEPROM
  void saveHomeToEEPROM();
  
  // Load home positions from EEPROM
  void loadHomeFromEEPROM();
  
  // Set current positions as home (in memory only)
  void setHomeFromCurrent();
  
  // Check if EEPROM contains valid calibration data
  bool hasValidCalibration() const;
  
  // Reset EEPROM to factory defaults
  void factoryReset();
  
  // ============================================================
  // DIAGNOSTICS & STATUS
  // ============================================================
  
  // Get current joint angles (for debugging/telemetry)
  void getJointAngles(float &base, float &shoulder, float &elbow, float &gripper) const;
  
  // Get current end-effector position (mm)
  void getCurrentPosition(float &x, float &y, float &z) const;
  
  // Validate if a position is within workspace constraints
  bool isPositionReachable(float x, float y, float z) const;

private:
  // Joint instances
  Joint base, shoulder, elbow, gripper;
  
  // Home/calibration positions (stored as int16_t for EEPROM compatibility)
  int16_t homeBase, homeShoulder, homeElbow, homeGripper;
  
  // ============================================================
  // INTERNAL HELPERS - Motion Profiling
  // ============================================================
  
  void initJoint(Joint &j, uint8_t pin, float initDeg);
  
  // Start trapezoidal motion profile for a joint
  void startTrapezoidalProfile(Joint &j, float newTargetDeg, float feedRateDegPerS);
  
  // Update joint position based on motion profile state
  void updateJoint(Joint &j, unsigned long now);
  
  // Calculate trapezoidal profile parameters
  void calculateProfileParameters(Joint &j, float delta, float velocity);
  
  // ============================================================
  // INTERNAL HELPERS - Math & Validation
  // ============================================================
  
  float clamp(float v, float a, float b) const;
  bool isWorkspaceValid(float x, float y, float z) const;
  
  // ============================================================
  // INTERNAL HELPERS - EEPROM
  // ============================================================
  
  void writeEEPROMValidation();
  bool checkEEPROMValidation() const;
};

#endif
