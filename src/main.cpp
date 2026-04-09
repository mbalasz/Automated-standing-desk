#include <Arduino.h>
#include <Preferences.h>
#include <TMCStepper.h>

// ── Motor 1 (left leg) — all pins on left side of ESP32 ─────────────────────
#define STEP_PIN_1   33
#define DIR_PIN_1    25
#define ENABLE_PIN_1 27
#define DIAG_PIN_1   34   // input-only GPIO

// ── Motor 2 (right leg) — all pins on right side of ESP32 ───────────────────
#define STEP_PIN_2    5
#define DIR_PIN_2    18
#define ENABLE_PIN_2 19
#define DIAG_PIN_2   21

// ── TMC2209 UART (Serial2) ────────────────────────────────────────────────────
#define TMC_SERIAL_RX_PIN 16
#define TMC_SERIAL_TX_PIN 17
#define TMC_UART_BAUD     115200

// ── Motor 2 direction inversion ───────────────────────────────────────────────
// Motors are mounted mirrored (shafts face outward). Toggle to 0 if one leg
// moves the wrong way — the driver inverts internally via shaft(true).
#define MOTOR_2_DIR_INVERTED 1

// ── TMC2209 UART addresses (set by MS1/MS2 pins on each driver board) ─────────
#define TMC_ADDR_MOTOR_1  0   // MS1=LOW,  MS2=LOW
#define TMC_ADDR_MOTOR_2  1   // MS1=HIGH, MS2=LOW

#define DESK_UP_PIN 14
#define DESK_UP_GROUND_PIN 26
#define DESK_DOWN_PIN 12
#define PRESET_1_PIN 25
#define PRESET_2_PIN 32
#define DIST_SENSOR_TRIG_PIN 22
#define DIST_SENSOR_ECHO_PIN 23

#define MOTION_STATE_UP 1
#define MOTION_STATE_DOWN -1
#define MOTION_STATE_DISABLED 0

#define PRESET_PREFS_NAMESPACE "preset"
#define DESK_PREFS_NAMESPACE "desk"
#define DESK_PREFS_HEIGHT_KEY "height"

#define STOPPED 0
#define RUNNING 1
#define DECELERATING 2
#define ACCELERATING 3

const double stepDelayMs = 0.1;
unsigned int accelerationSteps = 2200;
unsigned int currAccelerationSteps = 0;
unsigned int currDecelerationSteps = 0;
const unsigned int BACKTRACK_STEPS_AFTER_GOING_UP = 300;
const unsigned int BACKTRACK_STEPS_AFTER_GOING_DOWN = 60;
double maxSpeedDelayMs = 2;
double minSpeedDelayMs = 0.23;
double currSpeedDelayMs = maxSpeedDelayMs;
double delayDeltaPerMs = (maxSpeedDelayMs - minSpeedDelayMs) / accelerationSteps;
int prevLoopTimeMs = 0;

const unsigned int MOTOR_MAX_STEPS = 183000;
unsigned int maxDeskHeight = MOTOR_MAX_STEPS;
unsigned int minDeskHeight = 0;
int currDeskHeight = 0;
int currMotionState = STOPPED;
int currMotionDir = 0; // -1: moving down, 1: moving up, 0: not moving

// ── Homing constants ─────────────────────────────────────────────────────────
const double HOMING_SPEED_DELAY_MS = 1.5;
const unsigned int HOMING_TIMEOUT_MS = 30000; // 30s safety cutoff

// ── Stall / homing state (written from ISRs — must be volatile) ───────────────
volatile bool motor1Stalled = false;
volatile bool motor2Stalled = false;
volatile bool homingInProgress = false;
volatile bool motor1HomingDone = false;
volatile bool motor2HomingDone = false;

// ── Button state ─────────────────────────────────────────────────────────────
int moveUpButtonLastState = HIGH;
int moveDownButtonLastState = HIGH;

const int PRESET_BUTTONS_PINS[] = {32, 25};
int presetButtonStates[] = {HIGH, HIGH};
int presetButtonPressedTimeMs = -1;

unsigned long bothPresetsPressedSince = 0;
bool homingTriggered = false;

// ── TMC2209 driver objects ────────────────────────────────────────────────────
TMC2209Stepper driver1(&Serial2, 0.11f, TMC_ADDR_MOTOR_1);
TMC2209Stepper driver2(&Serial2, 0.11f, TMC_ADDR_MOTOR_2);

Preferences preferences;

// ── Preference helpers ────────────────────────────────────────────────────────

void intToString(unsigned int number, char* const str) {
  sprintf(str, "%u", number);
}

void storePreset(unsigned int presetNumber, unsigned int deskHeight) {
  Serial.println((String)"Storing preset: " + presetNumber + " with value: " + deskHeight);
  char presetNumberStr[3];
  intToString(presetNumber, presetNumberStr);
  preferences.begin(PRESET_PREFS_NAMESPACE);
  preferences.putUInt(presetNumberStr, deskHeight);
  preferences.end();
}

unsigned int readPreset(unsigned int presetNumber) {
  char presetNumberStr[3];
  intToString(presetNumber, presetNumberStr);
  preferences.begin(PRESET_PREFS_NAMESPACE);
  unsigned int presetValue = preferences.getUInt(presetNumberStr);
  preferences.end();
  Serial.println((String)"Retrieved preset: " + presetNumber + " with value: " + presetValue);
  return presetValue;
}

void storeDeskHeight(unsigned int height) {
  Serial.println((String)"Storing height: " + height);
  preferences.begin(DESK_PREFS_NAMESPACE);
  preferences.putUInt(DESK_PREFS_HEIGHT_KEY, height);
  preferences.end();
}

unsigned int readDeskHeight() {
  preferences.begin(DESK_PREFS_NAMESPACE);
  unsigned int height = preferences.getUInt(DESK_PREFS_HEIGHT_KEY);
  preferences.end();
  Serial.println((String)"Retrieved height: " + height);
  return height;
}

// ── Height boundaries ─────────────────────────────────────────────────────────

void setDeskHeightBoundaries(unsigned int minHeight, unsigned int maxHeight) {
  Serial.println((String)"Setting desk height boundaries to: (" + minHeight + "," + maxHeight + ")");
  minDeskHeight = max(0U, minHeight);
  maxDeskHeight = min(MOTOR_MAX_STEPS, maxHeight);
}

bool isWithinHeightBoundaries(int height) {
  return height >= (int)minDeskHeight && height <= (int)maxDeskHeight;
}

// ── DIR helpers ───────────────────────────────────────────────────────────────
// Motor 2 direction is inverted at the driver level via driver2.shaft(true),
// so both DIR pins always receive the same logical signal.

void setDirUp() {
  digitalWrite(DIR_PIN_1, HIGH);
  digitalWrite(DIR_PIN_2, HIGH);
}

void setDirDown() {
  digitalWrite(DIR_PIN_1, LOW);
  digitalWrite(DIR_PIN_2, LOW);
}

void setMotorsEnabled(bool enabled) {
  digitalWrite(ENABLE_PIN_1, enabled ? LOW : HIGH);
  digitalWrite(ENABLE_PIN_2, enabled ? LOW : HIGH);
}

// ── Motion control ────────────────────────────────────────────────────────────

void setMoveUp() {
  // Require more headroom than the post-stop backtrack, so the desk ends up higher than it started.
  if (currMotionState != STOPPED
      || !isWithinHeightBoundaries(currDeskHeight + MOTION_STATE_UP * (BACKTRACK_STEPS_AFTER_GOING_UP + 1))) {
    return;
  }
  Serial.println("setMoveUp");
  motor1Stalled = false;
  motor2Stalled = false;
  setMotorsEnabled(true);
  setDirUp();
  currMotionState = ACCELERATING;
  currMotionDir = MOTION_STATE_UP;
}

void setMoveDown() {
  // Require more clearance than the post-stop backtrack, so the desk ends up lower than it started.
  if (currMotionState != STOPPED
      || !isWithinHeightBoundaries(currDeskHeight + MOTION_STATE_DOWN * (BACKTRACK_STEPS_AFTER_GOING_DOWN + 1))) {
    return;
  }
  Serial.println("setMoveDown");
  motor1Stalled = false;
  motor2Stalled = false;
  setMotorsEnabled(true);
  setDirDown();
  currMotionState = ACCELERATING;
  currMotionDir = MOTION_STATE_DOWN;
}

void resetAcceleration() {
  currSpeedDelayMs = maxSpeedDelayMs;
  currAccelerationSteps = 0;
  currDecelerationSteps = 0;
}

void startDeceleration() {
  Serial.println((String)"Started deceleration at height: " + currDeskHeight);
  currDecelerationSteps = 0;
  currMotionState = DECELERATING;
}

bool approachingBoundary() {
  return !isWithinHeightBoundaries(currDeskHeight + currMotionDir * (int)(accelerationSteps * 2));
}

void makeStep() {
  digitalWrite(STEP_PIN_1, HIGH);
  digitalWrite(STEP_PIN_2, HIGH);
  delayMicroseconds((int)(stepDelayMs * 1000)); // 100 µs pulse
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(STEP_PIN_2, LOW);
  delayMicroseconds((int)(currSpeedDelayMs * 1000));
  currDeskHeight += currMotionDir;
}

void backtrack() {
  Serial.println("Backtracking");
  int backtrackSteps = currMotionDir == MOTION_STATE_DOWN
    ? BACKTRACK_STEPS_AFTER_GOING_DOWN
    : BACKTRACK_STEPS_AFTER_GOING_UP;
  currMotionDir *= -1;
  if (currMotionDir == MOTION_STATE_UP) {
    setDirUp();
  } else {
    setDirDown();
  }
  while (backtrackSteps-- > 0) {
    makeStep();
  }
}

void setStop(bool doBacktrack = true) {
  if (currMotionState == STOPPED) return;
  Serial.println((String)"setStop at height: " + currDeskHeight);
  if (doBacktrack) backtrack();
  currMotionState = STOPPED;
  currMotionDir = MOTION_STATE_DISABLED;
  resetAcceleration();
  storeDeskHeight(currDeskHeight);
  setDeskHeightBoundaries(0, MOTOR_MAX_STEPS);
  delay(500);
  setMotorsEnabled(false);
}

void rampSpeedUp(int deltaTime) {
  currSpeedDelayMs -= delayDeltaPerMs * deltaTime;
  currSpeedDelayMs = max(currSpeedDelayMs, minSpeedDelayMs);
}

void rampSpeedDown(int deltaTime) {
  currSpeedDelayMs += delayDeltaPerMs * deltaTime;
  currSpeedDelayMs = min(currSpeedDelayMs, maxSpeedDelayMs);
}

void handleEmergencyStop() {
  Serial.println("STALL DETECTED — emergency stop");
  motor1Stalled = false;
  motor2Stalled = false;
  setStop(false);
}

void setMoveToHeight(unsigned int height) {
  if (currMotionState != STOPPED) {
    return;
  }
  Serial.println((String)"Moving to height: " + height);
  if (currDeskHeight > (int)height) {
    setDeskHeightBoundaries(height, MOTOR_MAX_STEPS);
    setMoveDown();
  } else {
    setDeskHeightBoundaries(0, height);
    setMoveUp();
  }
}

// ── Homing ────────────────────────────────────────────────────────────────────

// Steps only the motors that have not yet stalled. No height tracking.
void makeHomingStep() {
  if (!motor1HomingDone) { digitalWrite(STEP_PIN_1, HIGH); }
  if (!motor2HomingDone) { digitalWrite(STEP_PIN_2, HIGH); }
  delayMicroseconds((int)(stepDelayMs * 1000));
  if (!motor1HomingDone) { digitalWrite(STEP_PIN_1, LOW); }
  if (!motor2HomingDone) { digitalWrite(STEP_PIN_2, LOW); }
  delayMicroseconds((int)(HOMING_SPEED_DELAY_MS * 1000));
}

void performHoming() {
  Serial.println("Homing: starting — both motors driving down at slow speed");
  homingInProgress = true;
  currMotionState = RUNNING; 
  motor1HomingDone = false;
  motor2HomingDone = false;

  setMotorsEnabled(true);
  setDirDown();

  unsigned long start = millis();
  while (!motor1HomingDone || !motor2HomingDone) {
    if (millis() - start > HOMING_TIMEOUT_MS) {
      Serial.println("Homing: TIMEOUT — check SGTHRS or wiring. Height NOT reset.");
      setMotorsEnabled(false);
      homingInProgress = false;
      return;
    }
    makeHomingStep();
  }

  homingInProgress = false;
  currDeskHeight = 0;
  setStop(false);
  Serial.println("Homing: done, height = 0");
}

// ── DIAG ISR handlers ─────────────────────────────────────────────────────────

void IRAM_ATTR onMotor1Stall() {
  if (homingInProgress) {
    motor1HomingDone = true;
  } else {
    // In normal operation both motors must stop together to keep the desk level.
    motor1Stalled = true;
    motor2Stalled = true;
  }
}

void IRAM_ATTR onMotor2Stall() {
  if (homingInProgress) {
    motor2HomingDone = true;
  } else {
    motor1Stalled = true;
    motor2Stalled = true;
  }
}

// ── TMC2209 initialization ────────────────────────────────────────────────────

void initDrivers() {
  Serial2.begin(TMC_UART_BAUD, SERIAL_8N1, TMC_SERIAL_RX_PIN, TMC_SERIAL_TX_PIN);
  delay(100);

  driver1.begin();
  driver2.begin();

  // RMS current in mA. Adjust to your motor spec.
  // 600 mA is conservative for NEMA 17 — increase if motors skip steps.
  driver1.rms_current(600);
  driver2.rms_current(600);

  driver1.toff(4);
  driver2.toff(4);
  driver1.blank_time(24);
  driver2.blank_time(24);

  driver1.microsteps(16);
  driver2.microsteps(16);

  // TCOOLTHRS: StallGuard activates when TSTEP < TCOOLTHRS (i.e. above this speed).
  // At full speed TSTEP ~7072, at slowest ~26000. 10000 excludes slow/acceleration phase.
  driver1.TCOOLTHRS(10000);
  driver2.TCOOLTHRS(10000);

  // SGTHRS: StallGuard sensitivity. Range 0–255. Higher = trips more easily.
  // START AT 10 (conservative). Tune empirically:
  //   1. Add Serial.print(driver1.SG_RESULT()) inside makeHomingStep() temporarily.
  //   2. Trigger homing; note SG_RESULT value as motor hits the hard stop.
  //   3. Set SGTHRS ≈ (255 - that_value) / 2. Typical range: 15–35.
  driver1.SGTHRS(20);
  driver2.SGTHRS(20);

  // Invert motor 2 direction at driver level (motors are mirrored).
#if MOTOR_2_DIR_INVERTED
  driver2.shaft(true);
#endif

  // Verify UART communication — genuine TMC2209 returns 0x21.
  // If either prints 0 or 0xFF, check wiring and MS1/MS2 address pins.
  Serial.print("Driver 1 version: 0x"); Serial.println(driver1.version(), HEX);
  Serial.print("Driver 2 version: 0x"); Serial.println(driver2.version(), HEX);
}

// ── Preset button logic ───────────────────────────────────────────────────────

void onPresetButtonDown(unsigned int presetNumber) {
  if (presetButtonStates[presetNumber] == HIGH) {
    presetButtonPressedTimeMs = millis();
    presetButtonStates[presetNumber] = LOW;
  }
}

void onPresetButtonUp(unsigned int presetNumber) {
  if (presetButtonStates[presetNumber] == LOW) {
    if (millis() - presetButtonPressedTimeMs > 2000) {
      storePreset(presetNumber + 1, currDeskHeight);
    } else {
      setMoveToHeight(readPreset(presetNumber + 1));
    }
    presetButtonStates[presetNumber] = HIGH;
  }
}

void checkPresetButtonStates() {
  bool btn0 = (digitalRead(PRESET_BUTTONS_PINS[0]) == LOW);
  bool btn1 = (digitalRead(PRESET_BUTTONS_PINS[1]) == LOW);

  // Both buttons held simultaneously for >3s → trigger homing
  if (btn0 && btn1) {
    if (bothPresetsPressedSince == 0) {
      bothPresetsPressedSince = millis();
    } else if (!homingTriggered && millis() - bothPresetsPressedSince > 3000) {
      homingTriggered = true;
      if (currMotionState == STOPPED) {
        performHoming();
      }
    }
    // Block individual preset actions while both are held
    return;
  }

  // Reset combo tracking when either button is released
  bothPresetsPressedSince = 0;
  homingTriggered = false;

  // Normal per-button preset logic
  for (int i = 0; i < 2; i++) {
    int state = digitalRead(PRESET_BUTTONS_PINS[i]);
    if (state == LOW) {
      onPresetButtonDown(i);
    } else {
      onPresetButtonUp(i);
    }
  }
}

void checkMoveButtonStates() {
  int moveUpButtonCurrState = digitalRead(DESK_UP_PIN);
  int moveDownButtonCurrState = digitalRead(DESK_DOWN_PIN);
  if (moveUpButtonCurrState == LOW && moveUpButtonLastState == HIGH) {
    setMoveUp();
  } else if (moveUpButtonCurrState == HIGH && moveUpButtonLastState == LOW) {
    startDeceleration();
  }

  if (moveDownButtonCurrState == LOW && moveDownButtonLastState == HIGH) {
    setMoveDown();
  } else if (moveDownButtonCurrState == HIGH && moveDownButtonLastState == LOW) {
    startDeceleration();
  }
  moveUpButtonLastState = moveUpButtonCurrState;
  moveDownButtonLastState = moveDownButtonCurrState;
}

// IMPORTANT: Only use this method as a one-off when desk height is miscalibrated.
// After resetting the height, comment out all invocations and reupload the code.
void resetDeskHeightToZero() {
  storeDeskHeight(0);
}

// DEBUG: moves motors continuously and prints SG_RESULT. Apply finger resistance to
// the shaft to see SG_RESULT drop and stall trigger. Comment out when done.
void debugTestStallGuard() {
  Serial.println("Debug: StallGuard test — apply resistance to shaft to trigger stall");
  currSpeedDelayMs = minSpeedDelayMs;
  currMotionDir = MOTION_STATE_UP;
  setDirUp();
  setMotorsEnabled(true);
  Serial.print("GCONF driver1: 0x"); Serial.println(driver1.GCONF(), HEX);
  Serial.print("GCONF driver2: 0x"); Serial.println(driver2.GCONF(), HEX);

  unsigned long lastPrint = 0;
  // Allow StallGuard to stabilize before checking for stalls
  for (int i = 0; i < 1000; i++)
  {
    makeStep();
    if (millis() - lastPrint > 200) {
      Serial.print("SG1: "); Serial.print(driver1.SG_RESULT());
      Serial.print("  SG2: "); Serial.println(driver2.SG_RESULT());
      lastPrint = millis();
    }
  }
  motor1Stalled = false;
  motor2Stalled = false;

  while (!motor1Stalled && !motor2Stalled) {
    makeStep();
    if (millis() - lastPrint > 200) {
      Serial.print("SG1: "); Serial.print(driver1.SG_RESULT());
      Serial.print("  SG2: "); Serial.print(driver2.SG_RESULT());
      Serial.print("  TSTEP: "); Serial.println(driver1.TSTEP());
      lastPrint = millis();
    }
  }

  setMotorsEnabled(false);
  currMotionDir = MOTION_STATE_DISABLED;
  Serial.print("Debug: stall detected — SG1: "); Serial.print(driver1.SG_RESULT());
  Serial.print("  SG2: "); Serial.println(driver2.SG_RESULT());
}

// DEBUG: moves each motor up then down by a fixed number of steps at slow speed.
// Call once from setup(), comment out when done.
void debugTestMotors() {
  const int TEST_STEPS = 500;
  Serial.println("Debug: motor test starting");
  currSpeedDelayMs = maxSpeedDelayMs;
  setMotorsEnabled(true);

  Serial.println("Debug: moving up");
  setDirUp();
  currMotionDir = MOTION_STATE_UP;
  for (int i = 0; i < TEST_STEPS; i++) makeStep();

  delay(500);

  Serial.println("Debug: moving down");
  setDirDown();
  currMotionDir = MOTION_STATE_DOWN;
  for (int i = 0; i < TEST_STEPS; i++) makeStep();

  setMotorsEnabled(false);
  currMotionDir = MOTION_STATE_DISABLED;
  Serial.println("Debug: motor test done");
}

void setup() {
  Serial.begin(921600);

  // Motor 1
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);

  // Motor 2
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);

  setMotorsEnabled(false);

  // DIAG pins (interrupt-driven stall detection)
  pinMode(DIAG_PIN_1, INPUT);
  pinMode(DIAG_PIN_2, INPUT);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_1), onMotor1Stall, RISING);
  attachInterrupt(digitalPinToInterrupt(DIAG_PIN_2), onMotor2Stall, RISING);

  // Button inputs
  pinMode(DESK_UP_PIN, INPUT_PULLUP);
  pinMode(DESK_DOWN_PIN, INPUT_PULLUP);
  pinMode(DESK_UP_GROUND_PIN, INPUT_PULLDOWN);
  pinMode(PRESET_1_PIN, INPUT_PULLUP);
  pinMode(PRESET_2_PIN, INPUT_PULLUP);

  // Distance sensor (pins initialized, not actively used)
  pinMode(DIST_SENSOR_ECHO_PIN, INPUT_PULLDOWN);
  pinMode(DIST_SENSOR_TRIG_PIN, OUTPUT);

  // TMC2209 UART init + StallGuard configuration
  initDrivers();

  prevLoopTimeMs = millis();

  // resetDeskHeightToZero();
  currDeskHeight = readDeskHeight();
  Serial.println((String)"Initial desk height: " + currDeskHeight);

  debugTestMotors();
  debugTestStallGuard();
}

void loop() {
  int now = millis();
  int deltaTime = now - prevLoopTimeMs;
  prevLoopTimeMs = now;

  if (motor1Stalled || motor2Stalled) {
    handleEmergencyStop();
    return;
  }

  checkPresetButtonStates();
  checkMoveButtonStates();

  if (currMotionState == STOPPED) return;

  if (currMotionState == RUNNING && approachingBoundary()) {
    startDeceleration();
  }

  if (!isWithinHeightBoundaries(currDeskHeight + currMotionDir)) {
    setStop();
    return;
  }

  switch (currMotionState) {
    case ACCELERATING:
      if (currAccelerationSteps < accelerationSteps
          && isWithinHeightBoundaries(currDeskHeight + currMotionDir * (int)accelerationSteps)) {
        rampSpeedUp(deltaTime);
        currAccelerationSteps++;
      } else {
        currMotionState = RUNNING;
      }
      break;

    case DECELERATING:
      if (currDecelerationSteps < currAccelerationSteps) {
        rampSpeedDown(deltaTime);
        currDecelerationSteps++;
      } else {
        setStop();
        return;
      }
      break;
  }

  makeStep();
}
