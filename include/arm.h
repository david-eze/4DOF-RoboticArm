#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Servo.h>

#define LINK_SHOULDER 100.0f
#define LINK_ELBOW    100.0f
#define LINK_HAND     50.0f

#define MIN_REACH     20.0f
#define MAX_REACH     180.0f
#define MIN_Z         -50.0f
#define MAX_Z         200.0f

#define PIN_BASE     3
#define PIN_SHOULDER 5
#define PIN_ELBOW    6
#define PIN_GRIPPER  9

#define EEPROM_ADDR_BASE      0
#define EEPROM_ADDR_SHOULDER  2
#define EEPROM_ADDR_ELBOW     4
#define EEPROM_ADDR_GRIPPER   6
#define EEPROM_ADDR_MAGIC     8
#define EEPROM_MAGIC_VALUE    0xA55A

#define DEFAULT_MAX_VELOCITY     120.0f
#define DEFAULT_MAX_ACCELERATION 300.0f
#define MIN_PROFILE_DURATION     20
#define SERVO_UPDATE_RATE        20

enum ProfileState {
  PROFILE_IDLE = 0,
  PROFILE_ACCEL,
  PROFILE_CONST_VEL,
  PROFILE_DECEL
};

struct Joint {
  Servo servo;
  uint8_t pin;
  
  float currentDeg;
  float targetDeg;
  
  float maxVelDegPerS;
  float maxAccDegPerS2;
  
  ProfileState state;
  unsigned long profileStartMs;
  unsigned long profileDurationMs;
  float startDeg;
  float peakVel;
  float accelTime;
  float decelTime;
  bool moving;
};

class Arm {
public:
  Arm();
  
  void begin();
  void update();
  
  bool inverseKinematics(float x, float y, float z, 
                         float &baseDeg, float &shoulderDeg, float &elbowDeg);
  
  void forwardKinematics(float baseDeg, float shoulderDeg, float elbowDeg,
                         float &x, float &y, float &z);
  
  bool moveToXYZ(float x, float y, float z, float feedRateMmPerS = 50.0f);
  void emergencyStop();
  bool isMoving() const;
  void moveToHome(float feedRateDegPerS = 30.0f);
  void setGripperPercent(uint8_t pct);
  
  void saveHomeToEEPROM();
  void loadHomeFromEEPROM();
  void setHomeFromCurrent();
  bool hasValidCalibration() const;
  void factoryReset();
  
  void getJointAngles(float &base, float &shoulder, float &elbow, float &gripper) const;
  void getCurrentPosition(float &x, float &y, float &z) const;
  bool isPositionReachable(float x, float y, float z) const;

private:
  Joint base, shoulder, elbow, gripper;
  int16_t homeBase, homeShoulder, homeElbow, homeGripper;
  
  void initJoint(Joint &j, uint8_t pin, float initDeg);
  void startTrapezoidalProfile(Joint &j, float newTargetDeg, float feedRateDegPerS);
  void updateJoint(Joint &j, unsigned long now);
  void calculateProfileParameters(Joint &j, float delta, float velocity);
  
  float clamp(float v, float a, float b) const;
  bool isWorkspaceValid(float x, float y, float z) const;
  
  void writeEEPROMValidation();
  bool checkEEPROMValidation() const;
};

#endif
