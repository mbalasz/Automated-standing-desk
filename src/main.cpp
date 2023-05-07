#include <Arduino.h>
#include <Preferences.h>

#define DESK_UP_PIN 12
#define DESK_DOWN_PIN 14
#define DESK_DOWN_GROUND_PIN 26
#define PRESET_1_PIN 25
#define PRESET_2_PIN 32

#define DIR_PIN 19
#define DIR_GROUND_PIN 18
#define STEP_PIN 4
#define MOTION_STATE_UP 1
#define MOTION_STATE_DOWN -1
#define MOTION_STATE_DISABLED 0

#define PRESET_PREFS_NAMESPACE "preset"
#define DESK_PREFS_NAMESPACE "desk"
#define DESK_PREFS_HEIGHT_KEY "height"

const double delayBasicMs = 0.1;
unsigned int accelerationTimeMs = 1800;
double maxSpeedDelayMs = 1.5;
double minSpeedDelayMs = 0.23;
double currSpeedDelayMs = maxSpeedDelayMs;
double delayDeltaPerMs = (maxSpeedDelayMs - minSpeedDelayMs) / accelerationTimeMs;
int prevLoopTimeMs = -1;

const unsigned int MOTOR_MAX_STEPS = 183000;
bool enabled = false;
unsigned int maxDeskHeight = MOTOR_MAX_STEPS;
unsigned int minDeskHeight = 0;
unsigned int currDeskHeight = 0;
int currMotionState = 0; // -1: moving down, 1: moving up, 0: not moving

int moveUpButtonLastState = HIGH;
int moveDownButtonLastState = HIGH;

const int PRESET_BUTTONS_PINS[] = {25, 32};
int presetButtonStates[] = {HIGH, HIGH};
int presetButtonLastState = HIGH;
int presetButtonPressedTimeMs = -1;

Preferences preferences;

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

void setDeskHeightBoundaries(unsigned int minHeight, unsigned int maxHeight) {
  Serial.println((String)"Setting desk height boundaries to: (" + minHeight + "," + maxHeight + ")");
  minDeskHeight = max(0U, minHeight);
  maxDeskHeight = min(MOTOR_MAX_STEPS, maxHeight);
}

void setMoveUp() {
  if (enabled) { 
    return; 
  }
  Serial.println("setMoveUp");
  digitalWrite(DIR_PIN, HIGH);
  enabled = true;
  currMotionState = MOTION_STATE_UP;
}

void setMoveDown() {
  if (enabled) { 
    return; 
  }
  Serial.println("setMoveDown");
  digitalWrite(DIR_PIN, LOW);
  enabled = true;
  currMotionState = MOTION_STATE_DOWN;
}

void setStop() {
  if (!enabled) { 
    return; 
  }
  Serial.println((String) "setStop at height: " + currDeskHeight);
  enabled = false;
  currMotionState = MOTION_STATE_DISABLED;
  currSpeedDelayMs = maxSpeedDelayMs;
  prevLoopTimeMs = -1;
  storeDeskHeight(currDeskHeight);
  setDeskHeightBoundaries(0, MOTOR_MAX_STEPS);
}

void setMoveToHeight(unsigned int height) {
  if (enabled) {
    // Ignore new actions when the desk is already moving.
    return;
  }
  Serial.println((String) "Moving to height: " + height);
  if (currDeskHeight > height) {
    setDeskHeightBoundaries(height, MOTOR_MAX_STEPS);
    setMoveDown();
  } else {
    setDeskHeightBoundaries(0, height);
    setMoveUp();
  }
}

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
  if (moveUpButtonCurrState == LOW && moveUpButtonLastState == HIGH) {
    setMoveUp();
    moveUpButtonLastState = LOW;
  } else if (moveUpButtonCurrState == HIGH && moveUpButtonLastState == LOW) {
    setStop();
    moveUpButtonLastState = HIGH;
  }

  int moveDownButtonCurrState = digitalRead(DESK_DOWN_PIN);
  if (moveDownButtonCurrState == LOW && moveDownButtonLastState == HIGH) {
    setMoveDown();
    moveDownButtonLastState = LOW;
  } else if (moveDownButtonCurrState == HIGH && moveDownButtonLastState == LOW) {
    setStop();
    moveDownButtonLastState = HIGH;
  }
}

bool isWithinHeightBoundaries(int height) {
  return height >= minDeskHeight && height <= maxDeskHeight;
}

void setup() {
  Serial.begin(9600);
  pinMode(DESK_UP_PIN, INPUT_PULLUP);
  pinMode(DESK_DOWN_PIN, INPUT_PULLUP);
  pinMode(DESK_DOWN_GROUND_PIN, INPUT_PULLDOWN);
  pinMode(PRESET_1_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(DIR_GROUND_PIN, INPUT_PULLDOWN);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(PRESET_2_PIN, INPUT_PULLUP);
  currDeskHeight = readDeskHeight();
  Serial.println((String)"Initial desk height: " + currDeskHeight);
}

void loop() {
  checkPresetButtonStates();
  checkMoveButtonStates();
  if (enabled && isWithinHeightBoundaries(currDeskHeight + currMotionState)) {
    int currTimeMs = millis();
    if (prevLoopTimeMs == -1) {
      prevLoopTimeMs = currTimeMs;
    } else if (currSpeedDelayMs > minSpeedDelayMs) {
      int elapsedTimeMs = currTimeMs - prevLoopTimeMs;
      currSpeedDelayMs -= delayDeltaPerMs * elapsedTimeMs;
      currSpeedDelayMs = max(currSpeedDelayMs, minSpeedDelayMs);
      prevLoopTimeMs = currTimeMs;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds((int) (delayBasicMs * 1000));
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds((int) (currSpeedDelayMs * 1000));
    currDeskHeight += currMotionState;
  } else if (enabled && !isWithinHeightBoundaries(currDeskHeight + currMotionState)) {
    setStop();
  }
}