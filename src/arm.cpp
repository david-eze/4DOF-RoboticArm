#include "Arduino.h"
#include "Arm.h"
#include <EEPROM.h>
#include <math.h>

// Conversion helpers
static inline float rad2deg(float r) { return r * 180.0f / PI; }
static inline float deg2rad(float d) { return d * PI / 180.0f; }

Arm::Arm() {
  homeBase = 90;
  homeShoulder = 90;
  homeElbow = 90;
  homeGripper = 90;
}

// ============================================================
// INITIALIZATION - Setup servos and load calibration
// ============================================================
void Arm::begin() {
  // Initialize all joints with safe parameters
  initJoint(base, PIN_BASE, (float)homeBase);
  initJoint(shoulder, PIN_SHOULDER, (float)homeShoulder);
  initJoint(elbow, PIN_ELBOW, (float)homeElbow);
  initJoint(gripper, PIN_GRIPPER, (float)homeGripper);

  // Load calibration from EEPROM if available
  if (hasValidCalibration()) {
    loadHomeFromEEPROM();
  } else {
    // Use factory defaults if no valid calibration
    writeEEPROMValidation();
  }

  // Apply home positions immediately (no motion profiling on init)
  base.currentDeg = (float)homeBase;
  base.targetDeg = base.currentDeg;
  base.servo.write((int)base.currentDeg);

  shoulder.currentDeg = (float)homeShoulder;
  shoulder.targetDeg = shoulder.currentDeg;
  shoulder.servo.write((int)shoulder.currentDeg);

  elbow.currentDeg = (float)homeElbow;
  elbow.targetDeg = elbow.currentDeg;
  elbow.servo.write((int)elbow.currentDeg);

  gripper.currentDeg = (float)homeGripper;
  gripper.targetDeg = gripper.currentDeg;
  gripper.servo.write((int)gripper.currentDeg);
}

// ============================================================
// JOINT INITIALIZATION - Configure motion constraints
// ============================================================
void Arm::initJoint(Joint &j, uint8_t pin, float initDeg) {
  j.pin = pin;
  j.servo.attach(pin);
  j.currentDeg = initDeg;
  j.targetDeg = initDeg;
  
  // Motion constraints - conservative for safety
  j.maxVelDegPerS = DEFAULT_MAX_VELOCITY;
  j.maxAccDegPerS2 = DEFAULT_MAX_ACCELERATION;
  
  // Initialize motion profile state
  j.state = PROFILE_IDLE;
  j.profileStartMs = 0;
  j.profileDurationMs = 0;
  j.startDeg = initDeg;
  j.peakVel = 0.0f;
  j.accelTime = 0.0f;
  j.decelTime = 0.0f;
  j.moving = false;
}

// ============================================================
// PERIODIC UPDATE - Non-blocking motion profiler
// Call this function every loop iteration for smooth motion
// ============================================================
void Arm::update() {
  unsigned long now = millis();
  updateJoint(base, now);
  updateJoint(shoulder, now);
  updateJoint(elbow, now);
  updateJoint(gripper, now);
}

// ============================================================
// JOINT UPDATE - Trapezoidal motion profile execution
// Implements: Acceleration -> Constant Velocity -> Deceleration
// ============================================================
void Arm::updateJoint(Joint &j, unsigned long now) {
  if (!j.moving) return;
  
  unsigned long elapsed = now - j.profileStartMs;
  float elapsedS = elapsed / 1000.0f; // Convert to seconds
  
  // Check if motion is complete
  if (elapsed >= j.profileDurationMs) {
    j.currentDeg = j.targetDeg;
    j.moving = false;
    j.state = PROFILE_IDLE;
  } else {
    // Calculate position based on current profile phase
    float position = 0.0f;
    
    switch (j.state) {
      case PROFILE_ACCEL:
        // Acceleration phase: pos = 0.5 * a * t^2 (with direction)
        if (elapsedS <= j.accelTime) {
          float accelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
          position = 0.5f * j.maxAccDegPerS2 * elapsedS * elapsedS * accelDir;
        } else {
          // Transition to constant velocity
          j.state = PROFILE_CONST_VEL;
          float accelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
          position = 0.5f * j.maxAccDegPerS2 * j.accelTime * j.accelTime * accelDir;
        }
        break;
        
      case PROFILE_CONST_VEL: {
        // Constant velocity phase: pos = pos_accel + v * t
        float accelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
        float accelDist = 0.5f * j.maxAccDegPerS2 * j.accelTime * j.accelTime * accelDir;
        float constVelElapsed = elapsedS - j.accelTime;
        
        if (constVelElapsed <= (j.profileDurationMs / 1000.0f - j.accelTime - j.decelTime)) {
          position = accelDist + j.peakVel * constVelElapsed;
        } else {
          // Transition to deceleration
          j.state = PROFILE_DECEL;
          position = accelDist + j.peakVel * (j.profileDurationMs / 1000.0f - j.accelTime - j.decelTime);
        }
        break;
      }
        
      case PROFILE_DECEL: {
        // Deceleration phase: pos = total_dist - 0.5 * a * (t_total - t)^2 (with direction)
        float totalDist = j.targetDeg - j.startDeg;
        float timeRemaining = (j.profileDurationMs / 1000.0f) - elapsedS;
        float decelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
        position = totalDist - (0.5f * j.maxAccDegPerS2 * timeRemaining * timeRemaining * decelDir);
        break;
      }
        
      default:
        // Fallback to linear interpolation
        float t = (float)elapsed / (float)j.profileDurationMs;
        position = (j.targetDeg - j.startDeg) * t;
        break;
    }
    
    // Apply calculated position
    j.currentDeg = j.startDeg + position;
  }
  
  // Apply to servo with safety clamping
  int out = (int)round(clamp(j.currentDeg, 0.0f, 180.0f));
  j.servo.write(out);
}

// ============================================================
// MATH HELPERS
// ============================================================
float Arm::clamp(float v, float a, float b) const {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

// ============================================================
// TRAPEZOIDAL PROFILE CALCULATION
// Calculates acceleration time, constant velocity time, and deceleration time
// ============================================================
void Arm::calculateProfileParameters(Joint &j, float delta, float velocity) {
  // Distance to travel (absolute value)
  float distance = fabs(delta);
  
  // Direction of movement (1.0 for positive, -1.0 for negative)
  float direction = (delta >= 0.0f) ? 1.0f : -1.0f;
  
  // Time to accelerate to target velocity
  j.accelTime = velocity / j.maxAccDegPerS2;
  
  // Distance covered during acceleration
  float accelDist = 0.5f * j.maxAccDegPerS2 * j.accelTime * j.accelTime;
  
  // Check if we can reach target velocity
  if (2.0f * accelDist > distance) {
    // Triangle profile (no constant velocity phase)
    j.accelTime = sqrt(distance / j.maxAccDegPerS2);
    j.decelTime = j.accelTime;
    j.peakVel = j.maxAccDegPerS2 * j.accelTime * direction;
  } else {
    // Trapezoidal profile
    j.decelTime = j.accelTime;
    j.peakVel = velocity * direction;
  }
  
  // Calculate total duration with division by zero protection
  float constVelTime = 0.0f;
  if (fabs(j.peakVel) > 0.001f) {
    constVelTime = (distance - 2.0f * accelDist) / fabs(j.peakVel);
  }
  
  float totalDurationS = j.accelTime + constVelTime + j.decelTime;
  
  j.profileDurationMs = (unsigned long)(totalDurationS * 1000.0f);
  if (j.profileDurationMs < MIN_PROFILE_DURATION) {
    j.profileDurationMs = MIN_PROFILE_DURATION;
  }
}

// ============================================================
// INVERSE KINEMATICS - Geometric solution for 3-DOF arm
// Mathematical derivation:
// 1. Base angle: atan2(y, x) - rotation in X-Y plane
// 2. Project target onto arm plane: r = sqrt(x^2 + y^2)
// 3. Solve 2-link arm for (r, z) using law of cosines:
//    cos(elbow) = (r^2 + z^2 - L1^2 - L2^2) / (2*L1*L2)
// 4. Shoulder angle: atan2(z, r) - atan2(L2*sin(elbow), L1 + L2*cos(elbow))
// ============================================================
bool Arm::inverseKinematics(float x, float y, float z, 
                        float &baseDeg, float &shoulderDeg, float &elbowDeg) {
  // Input validation
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
    return false;
  }
  
  // Workspace validation
  if (!isWorkspaceValid(x, y, z)) {
    return false;
  }
  
  // Calculate base rotation (azimuth)
  float baseRad = atan2f(y, x);
  
  // Project target onto the arm's vertical plane
  float r = sqrtf(x * x + y * y);
  
  // Check if target is within reach
  float maxReach = LINK_SHOULDER + LINK_ELBOW;
  float targetDist = sqrtf(r * r + z * z);
  
  if (targetDist > maxReach || targetDist < fabs(LINK_SHOULDER - LINK_ELBOW)) {
    return false; // Target out of reach
  }
  
  // Law of cosines for elbow angle
  float cosElbow = (r * r + z * z - LINK_SHOULDER * LINK_SHOULDER - LINK_ELBOW * LINK_ELBOW) / 
                    (2.0f * LINK_SHOULDER * LINK_ELBOW);
  
  // Clamp to valid range for acos
  cosElbow = clamp(cosElbow, -1.0f, 1.0f);
  
  // Calculate elbow angle (relative to straight line)
  float elbowRad = acosf(cosElbow);
  
  // Calculate shoulder angle
  float k1 = LINK_SHOULDER + LINK_ELBOW * cosf(elbowRad);
  float k2 = LINK_ELBOW * sinf(elbowRad);
  float shoulderRad = atan2f(z, r) - atan2f(k2, k1);
  
  // Convert to degrees
  baseDeg = rad2deg(baseRad);
  shoulderDeg = rad2deg(shoulderRad);
  elbowDeg = rad2deg(elbowRad);
  
  // Final validation of computed angles
  if (!isfinite(baseDeg) || !isfinite(shoulderDeg) || !isfinite(elbowDeg)) {
    return false;
  }
  
  return true;
}

// ============================================================
// FORWARD KINEMATICS - Compute XYZ from joint angles
// Useful for debugging and validation
// ============================================================
void Arm::forwardKinematics(float baseDeg, float shoulderDeg, float elbowDeg,
                        float &x, float &y, float &z) {
  // Convert to radians
  float baseRad = deg2rad(baseDeg);
  float shoulderRad = deg2rad(shoulderDeg);
  float elbowRad = deg2rad(elbowDeg);
  
  // Calculate arm configuration in vertical plane
  float totalAngle = shoulderRad + elbowRad;
  
  // End effector position in arm plane
  float r_arm = LINK_SHOULDER * cosf(shoulderRad) + LINK_ELBOW * cosf(totalAngle);
  float z_arm = LINK_SHOULDER * sinf(shoulderRad) + LINK_ELBOW * sinf(totalAngle);
  
  // Rotate by base angle to get XYZ coordinates
  x = r_arm * cosf(baseRad);
  y = r_arm * sinf(baseRad);
  z = z_arm;
}

// ============================================================
// TRAPEZOIDAL PROFILE STARTER - Begin smooth motion
// ============================================================
void Arm::startTrapezoidalProfile(Joint &j, float newTargetDeg, float feedRateDegPerS) {
  // Initialize profile parameters
  j.startDeg = j.currentDeg;
  j.targetDeg = newTargetDeg;
  
  // Calculate distance
  float delta = j.targetDeg - j.startDeg;
  float distance = fabs(delta);
  
  // Clamp velocity to safe limits
  float velocity = clamp(feedRateDegPerS, 1.0f, j.maxVelDegPerS);
  
  // Handle small movements or zero velocity
  if (velocity <= 0.0f || distance < 0.1f) {
    j.currentDeg = j.targetDeg;
    j.moving = false;
    j.state = PROFILE_IDLE;
    j.profileDurationMs = 0;
    return;
  }
  
  // Calculate trapezoidal profile parameters
  calculateProfileParameters(j, delta, velocity);
  
  // Initialize profile state
  j.profileStartMs = millis();
  j.state = PROFILE_ACCEL;
  j.moving = true;
}

// ============================================================
// WORKSPACE VALIDATION - Check if position is reachable
// ============================================================
bool Arm::isWorkspaceValid(float x, float y, float z) const {
  // Check height constraints
  if (z < MIN_Z || z > MAX_Z) {
    return false;
  }
  
  // Check radial distance from base
  float r = sqrtf(x * x + y * y);
  if (r < MIN_REACH || r > MAX_REACH) {
    return false;
  }
  
  // Check if within arm's physical reach
  float maxReach = LINK_SHOULDER + LINK_ELBOW;
  float minReach = fabs(LINK_SHOULDER - LINK_ELBOW);
  float targetDist = sqrtf(r * r + z * z);
  
  if (targetDist > maxReach || targetDist < minReach) {
    return false;
  }
  
  return true;
}

// ============================================================
// POSITION REACHABILITY CHECK - Public interface
// ============================================================
bool Arm::isPositionReachable(float x, float y, float z) const {
  return isWorkspaceValid(x, y, z);
}

// ============================================================
// MOVE TO XYZ - High-level motion command
// ============================================================
bool Arm::moveToXYZ(float x, float y, float z, float feedRateMmPerS) {
  // Compute inverse kinematics
  float baseDeg, shDeg, elDeg;
  if (!inverseKinematics(x, y, z, baseDeg, shDeg, elDeg)) {
    return false;
  }
  
  // Convert feed rate from mm/s to deg/s (approximate mapping)
  // This factor depends on your arm's geometry - tune as needed
  const float MM_TO_DEG = 0.8f;
  float feedDegPerS = clamp(feedRateMmPerS * MM_TO_DEG, 5.0f, DEFAULT_MAX_VELOCITY);
  
  // Start trapezoidal profiles for all joints
  startTrapezoidalProfile(base, baseDeg, feedDegPerS);
  startTrapezoidalProfile(shoulder, shDeg, feedDegPerS);
  startTrapezoidalProfile(elbow, elDeg, feedDegPerS);
  
  return true;
}

// ============================================================
// GRIPPER CONTROL - Open/close percentage
// ============================================================
void Arm::setGripperPercent(uint8_t pct) {
  pct = constrain(pct, 0, 100);
  // Map 0-100% to servo angle range (adjust for your gripper)
  float deg = 30.0f + (pct / 100.0f) * 120.0f; // 30° (closed) to 150° (open)
  startTrapezoidalProfile(gripper, deg, 180.0f);
}

// ============================================================
// EMERGENCY STOP - Immediate halt of all motion
// ============================================================
void Arm::emergencyStop() {
  base.moving = false;
  base.state = PROFILE_IDLE;
  
  shoulder.moving = false;
  shoulder.state = PROFILE_IDLE;
  
  elbow.moving = false;
  elbow.state = PROFILE_IDLE;
  
  gripper.moving = false;
  gripper.state = PROFILE_IDLE;
}

// ============================================================
// MOTION STATUS CHECK
// ============================================================
bool Arm::isMoving() const {
  return (base.moving || shoulder.moving || elbow.moving || gripper.moving);
}

// ============================================================
// EEPROM VALIDATION
// ============================================================
void Arm::writeEEPROMValidation() {
  EEPROM.put(EEPROM_ADDR_MAGIC, EEPROM_MAGIC_VALUE);
}

bool Arm::checkEEPROMValidation() const {
  uint16_t magic;
  EEPROM.get(EEPROM_ADDR_MAGIC, magic);
  return (magic == EEPROM_MAGIC_VALUE);
}

bool Arm::hasValidCalibration() const {
  return checkEEPROMValidation();
}

// ============================================================
// EEPROM SAVE - Store home positions with validation
// ============================================================
void Arm::saveHomeToEEPROM() {
  // Update home positions from current state
  homeBase = (int16_t)round(base.currentDeg);
  homeShoulder = (int16_t)round(shoulder.currentDeg);
  homeElbow = (int16_t)round(elbow.currentDeg);
  homeGripper = (int16_t)round(gripper.currentDeg);
  
  // Write to EEPROM
  EEPROM.put(EEPROM_ADDR_BASE, homeBase);
  EEPROM.put(EEPROM_ADDR_SHOULDER, homeShoulder);
  EEPROM.put(EEPROM_ADDR_ELBOW, homeElbow);
  EEPROM.put(EEPROM_ADDR_GRIPPER, homeGripper);
  
  // Write validation marker
  writeEEPROMValidation();
}

// ============================================================
// EEPROM LOAD - Retrieve home positions with validation
// ============================================================
void Arm::loadHomeFromEEPROM() {
  // Only load if validation marker is present
  if (!checkEEPROMValidation()) {
    return; // Use factory defaults
  }
  
  int16_t a, b, c, d;
  EEPROM.get(EEPROM_ADDR_BASE, a);
  EEPROM.get(EEPROM_ADDR_SHOULDER, b);
  EEPROM.get(EEPROM_ADDR_ELBOW, c);
  EEPROM.get(EEPROM_ADDR_GRIPPER, d);
  
  // Sanity checks - only load valid servo angles
  if (a >= 0 && a <= 180) homeBase = a;
  if (b >= 0 && b <= 180) homeShoulder = b;
  if (c >= 0 && c <= 180) homeElbow = c;
  if (d >= 0 && d <= 180) homeGripper = d;
}

// ============================================================
// HOME CALIBRATION - Set current position as home
// ============================================================
void Arm::setHomeFromCurrent() {
  homeBase = (int16_t)round(base.currentDeg);
  homeShoulder = (int16_t)round(shoulder.currentDeg);
  homeElbow = (int16_t)round(elbow.currentDeg);
  homeGripper = (int16_t)round(gripper.currentDeg);
}

// ============================================================
// FACTORY RESET - Clear EEPROM calibration
// ============================================================
void Arm::factoryReset() {
  // Reset to factory defaults
  homeBase = 90;
  homeShoulder = 90;
  homeElbow = 90;
  homeGripper = 90;
  
  // Clear validation marker to force defaults on next boot
  uint16_t invalidMagic = 0x0000;
  EEPROM.put(EEPROM_ADDR_MAGIC, invalidMagic);
}

// ============================================================
// MOVE TO HOME - Return to calibrated position
// ============================================================
void Arm::moveToHome(float feedRateDegPerS) {
  // Ensure EEPROM values are loaded
  loadHomeFromEEPROM();
  
  // Start smooth profiles to home positions
  startTrapezoidalProfile(base, (float)homeBase, feedRateDegPerS);
  startTrapezoidalProfile(shoulder, (float)homeShoulder, feedRateDegPerS);
  startTrapezoidalProfile(elbow, (float)homeElbow, feedRateDegPerS);
  startTrapezoidalProfile(gripper, (float)homeGripper, feedRateDegPerS);
}

// ============================================================
// DIAGNOSTICS - Get current joint angles
// ============================================================
void Arm::getJointAngles(float &baseAngle, float &shoulderAngle, 
                         float &elbowAngle, float &gripperAngle) const {
  baseAngle = base.currentDeg;
  shoulderAngle = shoulder.currentDeg;
  elbowAngle = elbow.currentDeg;
  gripperAngle = gripper.currentDeg;
}

// ============================================================
// DIAGNOSTICS - Get current end-effector position
// ============================================================
void Arm::getCurrentPosition(float &x, float &y, float &z) const {
  forwardKinematics(base.currentDeg, shoulder.currentDeg, 
                   elbow.currentDeg, x, y, z);
}
