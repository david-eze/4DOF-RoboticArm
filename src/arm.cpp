#include "Arduino.h"
#include "Arm.h"
#include <EEPROM.h>
#include <math.h>

static inline float rad2deg(float r) { return r * 180.0f / PI; }
static inline float deg2rad(float d) { return d * PI / 180.0f; }

Arm::Arm() {
  homeBase = 90;
  homeShoulder = 90;
  homeElbow = 90;
  homeGripper = 90;
}

void Arm::begin() {
  initJoint(base, PIN_BASE, (float)homeBase);
  initJoint(shoulder, PIN_SHOULDER, (float)homeShoulder);
  initJoint(elbow, PIN_ELBOW, (float)homeElbow);
  initJoint(gripper, PIN_GRIPPER, (float)homeGripper);

  if (hasValidCalibration()) {
    loadHomeFromEEPROM();
  } else {
    writeEEPROMValidation();
  }

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

void Arm::initJoint(Joint &j, uint8_t pin, float initDeg) {
  j.pin = pin;
  j.servo.attach(pin);
  j.currentDeg = initDeg;
  j.targetDeg = initDeg;
  
  j.maxVelDegPerS = DEFAULT_MAX_VELOCITY;
  j.maxAccDegPerS2 = DEFAULT_MAX_ACCELERATION;
  
  j.state = PROFILE_IDLE;
  j.profileStartMs = 0;
  j.profileDurationMs = 0;
  j.startDeg = initDeg;
  j.peakVel = 0.0f;
  j.accelTime = 0.0f;
  j.decelTime = 0.0f;
  j.moving = false;
}

void Arm::update() {
  unsigned long now = millis();
  updateJoint(base, now);
  updateJoint(shoulder, now);
  updateJoint(elbow, now);
  updateJoint(gripper, now);
}

void Arm::updateJoint(Joint &j, unsigned long now) {
  if (!j.moving) return;
  
  unsigned long elapsed = now - j.profileStartMs;
  float elapsedS = elapsed / 1000.0f;
  
  if (elapsed >= j.profileDurationMs) {
    j.currentDeg = j.targetDeg;
    j.moving = false;
    j.state = PROFILE_IDLE;
  } else {
    float position = 0.0f;
    
    switch (j.state) {
      case PROFILE_ACCEL:
        if (elapsedS <= j.accelTime) {
          float accelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
          position = 0.5f * j.maxAccDegPerS2 * elapsedS * elapsedS * accelDir;
        } else {
          j.state = PROFILE_CONST_VEL;
          float accelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
          position = 0.5f * j.maxAccDegPerS2 * j.accelTime * j.accelTime * accelDir;
        }
        break;
        
      case PROFILE_CONST_VEL: {
        float accelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
        float accelDist = 0.5f * j.maxAccDegPerS2 * j.accelTime * j.accelTime * accelDir;
        float constVelElapsed = elapsedS - j.accelTime;
        
        if (constVelElapsed <= (j.profileDurationMs / 1000.0f - j.accelTime - j.decelTime)) {
          position = accelDist + j.peakVel * constVelElapsed;
        } else {
          j.state = PROFILE_DECEL;
          position = accelDist + j.peakVel * (j.profileDurationMs / 1000.0f - j.accelTime - j.decelTime);
        }
        break;
      }
        
      case PROFILE_DECEL: {
        float totalDist = j.targetDeg - j.startDeg;
        float timeRemaining = (j.profileDurationMs / 1000.0f) - elapsedS;
        float decelDir = (j.peakVel >= 0.0f) ? 1.0f : -1.0f;
        position = totalDist - (0.5f * j.maxAccDegPerS2 * timeRemaining * timeRemaining * decelDir);
        break;
      }
        
      default:
        float t = (float)elapsed / (float)j.profileDurationMs;
        position = (j.targetDeg - j.startDeg) * t;
        break;
    }
    
    j.currentDeg = j.startDeg + position;
  }
  
  int out = (int)round(clamp(j.currentDeg, 0.0f, 180.0f));
  j.servo.write(out);
}

float Arm::clamp(float v, float a, float b) const {
  if (v < a) return a;
  if (v > b) return b;
  return v;
}

void Arm::calculateProfileParameters(Joint &j, float delta, float velocity) {
  float distance = fabs(delta);
  float direction = (delta >= 0.0f) ? 1.0f : -1.0f;
  
  j.accelTime = velocity / j.maxAccDegPerS2;
  float accelDist = 0.5f * j.maxAccDegPerS2 * j.accelTime * j.accelTime;
  
  if (2.0f * accelDist > distance) {
    j.accelTime = sqrt(distance / j.maxAccDegPerS2);
    j.decelTime = j.accelTime;
    j.peakVel = j.maxAccDegPerS2 * j.accelTime * direction;
  } else {
    j.decelTime = j.accelTime;
    j.peakVel = velocity * direction;
  }
  
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

bool Arm::inverseKinematics(float x, float y, float z, 
                            float &baseDeg, float &shoulderDeg, float &elbowDeg) {
  if (!isfinite(x) || !isfinite(y) || !isfinite(z)) {
    return false;
  }
  
  if (!isWorkspaceValid(x, y, z)) {
    return false;
  }
  
  float baseRad = atan2f(y, x);
  float r = sqrtf(x * x + y * y);
  
  float maxReach = LINK_SHOULDER + LINK_ELBOW;
  float targetDist = sqrtf(r * r + z * z);
  
  if (targetDist > maxReach || targetDist < fabs(LINK_SHOULDER - LINK_ELBOW)) {
    return false;
  }
  
  float cosElbow = (r * r + z * z - LINK_SHOULDER * LINK_SHOULDER - LINK_ELBOW * LINK_ELBOW) / 
                    (2.0f * LINK_SHOULDER * LINK_ELBOW);
  
  cosElbow = clamp(cosElbow, -1.0f, 1.0f);
  
  float elbowRad = acosf(cosElbow);
  
  float k1 = LINK_SHOULDER + LINK_ELBOW * cosf(elbowRad);
  float k2 = LINK_ELBOW * sinf(elbowRad);
  float shoulderRad = atan2f(z, r) - atan2f(k2, k1);
  
  baseDeg = rad2deg(baseRad);
  shoulderDeg = rad2deg(shoulderRad);
  elbowDeg = rad2deg(elbowRad);
  
  if (!isfinite(baseDeg) || !isfinite(shoulderDeg) || !isfinite(elbowDeg)) {
    return false;
  }
  
  return true;
}

void Arm::forwardKinematics(float baseDeg, float shoulderDeg, float elbowDeg,
                            float &x, float &y, float &z) {
  float baseRad = deg2rad(baseDeg);
  float shoulderRad = deg2rad(shoulderDeg);
  float elbowRad = deg2rad(elbowDeg);
  
  float totalAngle = shoulderRad + elbowRad;
  
  float r_arm = LINK_SHOULDER * cosf(shoulderRad) + LINK_ELBOW * cosf(totalAngle);
  float z_arm = LINK_SHOULDER * sinf(shoulderRad) + LINK_ELBOW * sinf(totalAngle);
  
  x = r_arm * cosf(baseRad);
  y = r_arm * sinf(baseRad);
  z = z_arm;
}

void Arm::startTrapezoidalProfile(Joint &j, float newTargetDeg, float feedRateDegPerS) {
  j.startDeg = j.currentDeg;
  j.targetDeg = newTargetDeg;
  
  float delta = j.targetDeg - j.startDeg;
  float distance = fabs(delta);
  
  float velocity = clamp(feedRateDegPerS, 1.0f, j.maxVelDegPerS);
  
  if (velocity <= 0.0f || distance < 0.1f) {
    j.currentDeg = j.targetDeg;
    j.moving = false;
    j.state = PROFILE_IDLE;
    j.profileDurationMs = 0;
    return;
  }
  
  calculateProfileParameters(j, delta, velocity);
  
  j.profileStartMs = millis();
  j.state = PROFILE_ACCEL;
  j.moving = true;
}

bool Arm::isWorkspaceValid(float x, float y, float z) const {
  if (z < MIN_Z || z > MAX_Z) {
    return false;
  }
  
  float r = sqrtf(x * x + y * y);
  if (r < MIN_REACH || r > MAX_REACH) {
    return false;
  }
  
  float maxReach = LINK_SHOULDER + LINK_ELBOW;
  float minReach = fabs(LINK_SHOULDER - LINK_ELBOW);
  float targetDist = sqrtf(r * r + z * z);
  
  if (targetDist > maxReach || targetDist < minReach) {
    return false;
  }
  
  return true;
}

bool Arm::isPositionReachable(float x, float y, float z) const {
  return isWorkspaceValid(x, y, z);
}

bool Arm::moveToXYZ(float x, float y, float z, float feedRateMmPerS) {
  float baseDeg, shDeg, elDeg;
  if (!inverseKinematics(x, y, z, baseDeg, shDeg, elDeg)) {
    return false;
  }
  
  const float MM_TO_DEG = 0.8f;
  float feedDegPerS = clamp(feedRateMmPerS * MM_TO_DEG, 5.0f, DEFAULT_MAX_VELOCITY);
  
  startTrapezoidalProfile(base, baseDeg, feedDegPerS);
  startTrapezoidalProfile(shoulder, shDeg, feedDegPerS);
  startTrapezoidalProfile(elbow, elDeg, feedDegPerS);
  
  return true;
}

void Arm::setGripperPercent(uint8_t pct) {
  pct = constrain(pct, 0, 100);
  float deg = 30.0f + (pct / 100.0f) * 120.0f;
  startTrapezoidalProfile(gripper, deg, 180.0f);
}

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

bool Arm::isMoving() const {
  return (base.moving || shoulder.moving || elbow.moving || gripper.moving);
}

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

void Arm::saveHomeToEEPROM() {
  homeBase = (int16_t)round(base.currentDeg);
  homeShoulder = (int16_t)round(shoulder.currentDeg);
  homeElbow = (int16_t)round(elbow.currentDeg);
  homeGripper = (int16_t)round(gripper.currentDeg);
  
  EEPROM.put(EEPROM_ADDR_BASE, homeBase);
  EEPROM.put(EEPROM_ADDR_SHOULDER, homeShoulder);
  EEPROM.put(EEPROM_ADDR_ELBOW, homeElbow);
  EEPROM.put(EEPROM_ADDR_GRIPPER, homeGripper);
  
  writeEEPROMValidation();
}

void Arm::loadHomeFromEEPROM() {
  if (!checkEEPROMValidation()) {
    return;
  }
  
  int16_t a, b, c, d;
  EEPROM.get(EEPROM_ADDR_BASE, a);
  EEPROM.get(EEPROM_ADDR_SHOULDER, b);
  EEPROM.get(EEPROM_ADDR_ELBOW, c);
  EEPROM.get(EEPROM_ADDR_GRIPPER, d);
  
  if (a >= 0 && a <= 180) homeBase = a;
  if (b >= 0 && b <= 180) homeShoulder = b;
  if (c >= 0 && c <= 180) homeElbow = c;
  if (d >= 0 && d <= 180) homeGripper = d;
}

void Arm::setHomeFromCurrent() {
  homeBase = (int16_t)round(base.currentDeg);
  homeShoulder = (int16_t)round(shoulder.currentDeg);
  homeElbow = (int16_t)round(elbow.currentDeg);
  homeGripper = (int16_t)round(gripper.currentDeg);
}

void Arm::factoryReset() {
  homeBase = 90;
  homeShoulder = 90;
  homeElbow = 90;
  homeGripper = 90;
  
  uint16_t invalidMagic = 0x0000;
  EEPROM.put(EEPROM_ADDR_MAGIC, invalidMagic);
}

void Arm::moveToHome(float feedRateDegPerS) {
  loadHomeFromEEPROM();
  
  startTrapezoidalProfile(base, (float)homeBase, feedRateDegPerS);
  startTrapezoidalProfile(shoulder, (float)homeShoulder, feedRateDegPerS);
  startTrapezoidalProfile(elbow, (float)homeElbow, feedRateDegPerS);
  startTrapezoidalProfile(gripper, (float)homeGripper, feedRateDegPerS);
}

void Arm::getJointAngles(float &baseAngle, float &shoulderAngle, 
                         float &elbowAngle, float &gripperAngle) const {
  baseAngle = base.currentDeg;
  shoulderAngle = shoulder.currentDeg;
  elbowAngle = elbow.currentDeg;
  gripperAngle = gripper.currentDeg;
}

void Arm::getCurrentPosition(float &x, float &y, float &z) const {
  forwardKinematics(base.currentDeg, shoulder.currentDeg, 
                   elbow.currentDeg, x, y, z);
}
