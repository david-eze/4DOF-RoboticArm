#include <Arduino.h>
#include "Arm.h"

Arm arm;

#define SERIAL_BUFFER_SIZE 128
#define COMMAND_TIMEOUT_MS 1000

static char lineBuf[SERIAL_BUFFER_SIZE];
static uint8_t linePos = 0;
static unsigned long lastCommandTime = 0;

static float parseCoordinate(const char *str, char axis) {
  const char *p = str;
  while (*p) {
    if (*p == axis || *p == tolower(axis)) {
      return atof(p + 1);
    }
    p++;
  }
  return NAN;
}

static bool validateCoordinates(float x, float y, float z) {
  if (isnan(x) || isnan(y) || isnan(z)) {
    return false;
  }
  
  if (fabs(x) > 500.0f || fabs(y) > 500.0f || fabs(z) > 500.0f) {
    return false;
  }
  
  return true;
}

static bool handleG0(const char *str) {
  float x = parseCoordinate(str, 'X');
  float y = parseCoordinate(str, 'Y');
  float z = parseCoordinate(str, 'Z');
  float f = parseCoordinate(str, 'F');
  
  if (isnan(f)) f = 50.0f;
  
  if (!validateCoordinates(x, y, z)) {
    Serial.println("ERR Invalid or missing coordinates");
    return false;
  }
  
  if (f <= 0.0f || f > 500.0f) {
    Serial.println("ERR Invalid feed rate (use 1-500 mm/s)");
    return false;
  }
  
  if (!arm.moveToXYZ(x, y, z, f)) {
    Serial.println("ERR Target unreachable or workspace violation");
    return false;
  }
  
  Serial.println("OK Motion started");
  return true;
}

static bool handleGrip(const char *str) {
  float pct = parseCoordinate(str, 'G');
  
  if (isnan(pct)) {
    Serial.println("ERR Missing gripper percentage");
    return false;
  }
  
  if (pct < 0.0f || pct > 100.0f) {
    Serial.println("ERR Gripper percentage must be 0-100");
    return false;
  }
  
  arm.setGripperPercent((uint8_t)pct);
  Serial.println("OK Gripper set");
  return true;
}

static void handleStatus() {
  float base, shoulder, elbow, gripper;
  arm.getJointAngles(base, shoulder, elbow, gripper);
  
  float x, y, z;
  arm.getCurrentPosition(x, y, z);
  
  bool moving = arm.isMoving();
  
  Serial.println("=== ARM STATUS ===");
  Serial.print("Joints: B="); Serial.print(base, 1);
  Serial.print(" S="); Serial.print(shoulder, 1);
  Serial.print(" E="); Serial.print(elbow, 1);
  Serial.print(" G="); Serial.println(gripper, 1);
  
  Serial.print("Position: X="); Serial.print(x, 1);
  Serial.print(" Y="); Serial.print(y, 1);
  Serial.print(" Z="); Serial.println(z, 1);
  
  Serial.print("Moving: "); Serial.println(moving ? "YES" : "NO");
  
  Serial.print("Calibration: "); 
  Serial.println(arm.hasValidCalibration() ? "VALID" : "NONE");
  Serial.println("==================");
}

static void handleHelp() {
  Serial.println("=== AVAILABLE COMMANDS ===");
  Serial.println("G0 X[x] Y[y] Z[z] F[f] - Move to coordinates (mm)");
  Serial.println("HOME                   - Move to home position");
  Serial.println("SAVE                   - Save current as home");
  Serial.println("LOAD                   - Load home from EEPROM");
  Serial.println("RESET                  - Factory reset EEPROM");
  Serial.println("STOP                   - Emergency stop");
  Serial.println("STATUS                 - Show current status");
  Serial.println("GRIP [0-100]           - Set gripper (0=closed, 100=open)");
  Serial.println("HELP                   - Show this help");
  Serial.println("=========================");
}

static void handleLine(const char *str) {
  while (*str == ' ' || *str == '\t') str++;
  
  if (*str == '\0') return;
  
  lastCommandTime = millis();
  
  if (strncmp(str, "G0", 2) == 0) {
    handleG0(str);
  } else if (strncmp(str, "G1", 2) == 0) {
    Serial.println("ERR G1 not implemented (reserved for future)");
  } else if (strcmp(str, "HOME") == 0) {
    arm.moveToHome(30.0f);
    Serial.println("OK Moving to home");
  } else if (strcmp(str, "SAVE") == 0) {
    arm.setHomeFromCurrent();
    arm.saveHomeToEEPROM();
    Serial.println("OK Home position saved");
  } else if (strcmp(str, "LOAD") == 0) {
    arm.loadHomeFromEEPROM();
    Serial.println("OK Home position loaded");
  } else if (strcmp(str, "RESET") == 0) {
    arm.factoryReset();
    Serial.println("OK Factory reset complete");
  } else if (strcmp(str, "STOP") == 0) {
    arm.emergencyStop();
    Serial.println("OK Emergency stop activated");
  } else if (strcmp(str, "STATUS") == 0) {
    handleStatus();
  } else if (strncmp(str, "GRIP", 4) == 0) {
    handleGrip(str);
  } else if (strcmp(str, "HELP") == 0) {
    handleHelp();
  } else {
    Serial.println("ERR Unknown command (type HELP for usage)");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  
  Serial.println("========================================");
  Serial.println("4-DOF Robotic Arm Controller");
  Serial.println("Professional-Grade Firmware v1.0");
  Serial.println("========================================");
  
  arm.begin();
  
  Serial.println("System ready");
  Serial.println("Type HELP for available commands");
  Serial.println("");
  
  lastCommandTime = millis();
}

void loop() {
  arm.update();
  
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    if (c == '\r') continue;
    
    if (c == '\n' || linePos >= SERIAL_BUFFER_SIZE - 1) {
      lineBuf[linePos] = '\0';
      
      if (linePos > 0) {
        handleLine(lineBuf);
      }
      
      linePos = 0;
    } else {
      lineBuf[linePos++] = c;
    }
  }
  
  if (millis() - lastCommandTime > 60000UL) {
  }
}
