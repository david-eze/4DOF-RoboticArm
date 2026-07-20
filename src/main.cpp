#include <Arduino.h>
#include "Arm.h"

// ============================================================
// GLOBAL INSTANCES
// ============================================================
Arm arm;

// ============================================================
// SERIAL COMMAND PARSER - Non-blocking, robust parsing
// ============================================================

#define SERIAL_BUFFER_SIZE 128
#define COMMAND_TIMEOUT_MS 1000

static char lineBuf[SERIAL_BUFFER_SIZE];
static uint8_t linePos = 0;
static unsigned long lastCommandTime = 0;

// ============================================================
// COMMAND PROTOCOL
// ============================================================
// G0 X100 Y50 Z120 F50  - Move to XYZ coordinates (mm) with feed rate
// HOME                   - Move to calibrated home position
// SAVE                   - Save current position as home to EEPROM
// LOAD                   - Load home position from EEPROM
// RESET                  - Factory reset EEPROM
// STOP                   - Emergency stop (immediate halt)
// STATUS                 - Get current joint angles and position
// GRIP 0-100            - Set gripper open percentage (0=closed, 100=open)
// HELP                   - Show available commands
// ============================================================

// Parse float value from command string (e.g., "X100" -> 100.0)
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

// Validate coordinate values
static bool validateCoordinates(float x, float y, float z) {
  if (isnan(x) || isnan(y) || isnan(z)) {
    return false;
  }
  
  // Basic range checks (workspace validation handled by Arm class)
  if (fabs(x) > 500.0f || fabs(y) > 500.0f || fabs(z) > 500.0f) {
    return false;
  }
  
  return true;
}

// Handle G0 command - Move to XYZ
static bool handleG0(const char *str) {
  float x = parseCoordinate(str, 'X');
  float y = parseCoordinate(str, 'Y');
  float z = parseCoordinate(str, 'Z');
  float f = parseCoordinate(str, 'F');
  
  // Default feed rate if not specified
  if (isnan(f)) f = 50.0f;
  
  // Validate coordinates
  if (!validateCoordinates(x, y, z)) {
    Serial.println("ERR Invalid or missing coordinates");
    return false;
  }
  
  // Validate feed rate
  if (f <= 0.0f || f > 500.0f) {
    Serial.println("ERR Invalid feed rate (use 1-500 mm/s)");
    return false;
  }
  
  // Attempt motion
  if (!arm.moveToXYZ(x, y, z, f)) {
    Serial.println("ERR Target unreachable or workspace violation");
    return false;
  }
  
  Serial.println("OK Motion started");
  return true;
}

// Handle GRIP command - Gripper control
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

// Handle STATUS command - Report current state
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

// Handle HELP command - Show available commands
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

// Main command handler
static void handleLine(const char *str) {
  // Trim whitespace
  while (*str == ' ' || *str == '\t') str++;
  
  // Skip empty lines
  if (*str == '\0') return;
  
  // Update command timestamp
  lastCommandTime = millis();
  
  // Command dispatch
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

// ============================================================
// SETUP - Initialize system
// ============================================================
void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) { ; } // Wait for serial port to connect
  
  Serial.println("========================================");
  Serial.println("4-DOF Robotic Arm Controller");
  Serial.println("Professional-Grade Firmware v1.0");
  Serial.println("========================================");
  
  // Initialize arm controller
  arm.begin();
  
  Serial.println("System ready");
  Serial.println("Type HELP for available commands");
  Serial.println("");
  
  lastCommandTime = millis();
}

// ============================================================
// MAIN LOOP - Non-blocking state machine
// ============================================================
void loop() {
  // Periodic updates for motion profiling (call every iteration)
  arm.update();
  
  // Non-blocking serial command processing
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    // Handle line endings
    if (c == '\r') continue; // Ignore carriage return
    
    if (c == '\n' || linePos >= SERIAL_BUFFER_SIZE - 1) {
      // Null-terminate and process command
      lineBuf[linePos] = '\0';
      
      if (linePos > 0) {
        handleLine(lineBuf);
      }
      
      linePos = 0;
    } else {
      // Buffer character
      lineBuf[linePos++] = c;
    }
  }
  
  // Optional: Watchdog for command timeout (can be expanded)
  if (millis() - lastCommandTime > 60000UL) {
    // No commands for 60 seconds - could enter idle mode
    // Currently just a placeholder for future power management
  }
}
