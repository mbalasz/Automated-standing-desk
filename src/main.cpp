#include <Arduino.h>
#include <Preferences.h>
#include "FastAccelStepper.h"

// --- Pin Definitions ---
#define DESK_UP_PIN             14
#define DESK_UP_GROUND_PIN      26
#define DESK_DOWN_PIN           12
#define PRESET_1_PIN            25
#define PRESET_2_PIN            32
#define DIR_PIN                 19
#define ENABLE_PIN              18
#define STEP_PIN                4
#define DIST_SENSOR_TRIG_PIN    22
#define DIST_SENSOR_ECHO_PIN    23

// --- Motor Parameters ---
// Driver: DM542Y at 1600 steps/rev (1/8 microstepping, SW5-SW8 DIP switches).
// Motor:  1.8°/step = 200 full steps/rev × 8 = 1600 steps/rev.
//
// Speed formula: MOTOR_MAX_SPEED_HZ / 1600 steps/rev / 3 rev/cm
//   e.g. 4347 steps/s → ~0.91 cm/s → ~33 s for a 30 cm range (conservative)
//
// TODO: raise MOTOR_MAX_SPEED_HZ to ~14000 for a more typical desk speed of ~3 cm/s (~10 s for 30 cm).
#define MOTOR_MAX_SPEED_HZ      4347   // steps/s at full speed
#define MOTOR_ACCELERATION      4000   // steps/s² — ramp steepness (higher = faster ramp)
#define MOTOR_MAX_STEPS         183000 // TODO: recalibrate by running desk end-to-end — measured at 2000 steps/rev, now invalid at 1600

// Small reverse move after stopping to release mechanical tension in the leadscrew.
#define BACKTRACK_STEPS_UP      300
#define BACKTRACK_STEPS_DOWN    60

// --- NVS Namespaces ---
#define PRESET_PREFS_NAMESPACE  "preset"
#define DESK_PREFS_NAMESPACE    "desk"
#define DESK_PREFS_HEIGHT_KEY   "height"

enum MoveDirection { IDLE, UP, DOWN };

// --- Globals ---
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;
Preferences preferences;

MoveDirection moveDirection = IDLE;
bool wasRunning = false;

int moveUpButtonLastState = HIGH;
int moveDownButtonLastState = HIGH;
const int PRESET_BUTTONS_PINS[] = {PRESET_2_PIN, PRESET_1_PIN};
int presetButtonStates[] = {HIGH, HIGH};
int presetButtonPressedTimeMs = -1;

// --- NVS Helpers ---
void intToString(unsigned int n, char* str) {
  sprintf(str, "%u", n);
}

void storePreset(unsigned int presetNumber, int32_t height) {
  Serial.println((String)"Storing preset " + presetNumber + " = " + height);
  char key[3];
  intToString(presetNumber, key);
  preferences.begin(PRESET_PREFS_NAMESPACE);
  preferences.putUInt(key, (unsigned int)height);
  preferences.end();
}

int32_t readPreset(unsigned int presetNumber) {
  char key[3];
  intToString(presetNumber, key);
  preferences.begin(PRESET_PREFS_NAMESPACE);
  int32_t val = preferences.getUInt(key);
  preferences.end();
  Serial.println((String)"Retrieved preset " + presetNumber + " = " + val);
  return val;
}

void storeDeskHeight(int32_t height) {
  Serial.println((String)"Storing height: " + height);
  preferences.begin(DESK_PREFS_NAMESPACE);
  preferences.putUInt(DESK_PREFS_HEIGHT_KEY, (unsigned int)height);
  preferences.end();
}

int32_t readDeskHeight() {
  preferences.begin(DESK_PREFS_NAMESPACE);
  int32_t height = preferences.getUInt(DESK_PREFS_HEIGHT_KEY);
  preferences.end();
  Serial.println((String)"Retrieved height: " + height);
  return height;
}

// IMPORTANT: One-off calibration only. Uncomment call in setup(), upload, re-comment, re-upload.
void resetDeskHeightToZero() {
  storeDeskHeight(0);
}

// --- Motion ---

// Called whenever the motor finishes any move (button release or preset arrival).
// Applies a short reverse (backtrack) to release leadscrew tension, then persists height.
void onMotorStop() {
  int backtrackSteps = (moveDirection == UP) ? -BACKTRACK_STEPS_UP : BACKTRACK_STEPS_DOWN;
  Serial.println((String)"Backtracking " + backtrackSteps + " steps");
  stepper->move(backtrackSteps, /*blocking=*/true);
  int32_t finalHeight = stepper->getCurrentPosition();
  Serial.println((String)"Stopped at height: " + finalHeight);
  storeDeskHeight(finalHeight);
  moveDirection = IDLE;
}

void moveUp() {
  if (stepper->isRunning()) return;
  Serial.println("moveUp");
  moveDirection = UP;
  stepper->moveTo(MOTOR_MAX_STEPS);
}

void moveDown() {
  if (stepper->isRunning()) return;
  Serial.println("moveDown");
  moveDirection = DOWN;
  stepper->moveTo(0);
}

void moveToHeight(int32_t target) {
  if (stepper->isRunning()) return;
  target = constrain(target, 0, MOTOR_MAX_STEPS);
  int32_t current = stepper->getCurrentPosition();
  if (target == current) return;
  Serial.println((String)"moveToHeight: " + target);
  moveDirection = (target > current) ? UP : DOWN;
  stepper->moveTo(target);
}

// --- Button Handlers ---
void checkPresetButtonStates() {
  for (int i = 0; i < 2; i++) {
    int state = digitalRead(PRESET_BUTTONS_PINS[i]);
    if (state == LOW && presetButtonStates[i] == HIGH) {
      presetButtonPressedTimeMs = millis();
      presetButtonStates[i] = LOW;
    } else if (state == HIGH && presetButtonStates[i] == LOW) {
      if (millis() - presetButtonPressedTimeMs > 2000) {
        storePreset(i + 1, stepper->getCurrentPosition());
      } else {
        moveToHeight(readPreset(i + 1));
      }
      presetButtonStates[i] = HIGH;
    }
  }
}

void checkMoveButtonStates() {
  int upState   = digitalRead(DESK_UP_PIN);
  int downState = digitalRead(DESK_DOWN_PIN);

  if (upState == LOW && moveUpButtonLastState == HIGH)         moveUp();
  else if (upState == HIGH && moveUpButtonLastState == LOW)    stepper->stopMove();

  if (downState == LOW && moveDownButtonLastState == HIGH)     moveDown();
  else if (downState == HIGH && moveDownButtonLastState == LOW) stepper->stopMove();

  moveUpButtonLastState   = upState;
  moveDownButtonLastState = downState;
}

// --- Arduino Entry Points ---
void setup() {
  Serial.begin(921600);

  pinMode(DESK_UP_PIN,          INPUT_PULLUP);
  pinMode(DESK_DOWN_PIN,        INPUT_PULLUP);
  pinMode(DESK_UP_GROUND_PIN,   INPUT_PULLDOWN);
  pinMode(PRESET_1_PIN,         INPUT_PULLUP);
  pinMode(PRESET_2_PIN,         INPUT_PULLUP);
  pinMode(DIST_SENSOR_ECHO_PIN, INPUT_PULLDOWN);
  pinMode(DIST_SENSOR_TRIG_PIN, OUTPUT);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  // Motor power consumption: 6W disabled, 9.5W with holding torque.
  // Disable after 500ms to release torque; savings are minor but avoids long-term miscalibration.
  stepper->setDelayToDisable(500);
  stepper->setSpeedInHz(MOTOR_MAX_SPEED_HZ);
  stepper->setAcceleration(MOTOR_ACCELERATION);

  // resetDeskHeightToZero();
  int32_t savedHeight = readDeskHeight();
  stepper->setCurrentPosition(savedHeight);
  Serial.println((String)"Initial desk height: " + savedHeight);
}

void loop() {
  checkPresetButtonStates();
  checkMoveButtonStates();

  // Detect motor-stop transitions to trigger backtrack and height persistence.
  bool isRunning = stepper->isRunning();
  if (wasRunning && !isRunning) {
    onMotorStop();
  }
  wasRunning = isRunning;
}
