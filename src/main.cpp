#include <Arduino.h>
#include <Preferences.h>

#define DESK_UP_PIN 12
#define DESK_DOWN_PIN 14
#define DESK_DOWN_GROUND_PIN 26
#define PRESET_1_PIN 25
#define PRESET_2_PIN 32
#define DIR_PIN 19
#define ENABLE_PIN 18
#define STEP_PIN 4
#define DIST_SENSOR_TRIG_PIN 22
#define DIST_SENSOR_ECHO_PIN 23

#define MOTION_STATE_UP 1
#define MOTION_STATE_DOWN -1
#define MOTION_STATE_DISABLED 0

#define PRESET_PREFS_NAMESPACE "preset"
#define DESK_PREFS_NAMESPACE "desk"
#define DESK_PREFS_HEIGHT_KEY "height"

#define DIST_IDLE 0
#define DIST_TRIGGER_SENT 1
#define DIST_TIMER_STARTED 2

const double delayBasicMs = 0.1;
unsigned int accelerationTimeMs = 1800;
double maxSpeedDelayMs = 1.5;
double minSpeedDelayMs = 0.23;
double currSpeedDelayMs = maxSpeedDelayMs;
double delayDeltaPerMs = (maxSpeedDelayMs - minSpeedDelayMs) / accelerationTimeMs;
int prevLoopTimeMs = -1;

int distanceTriggerState = DIST_IDLE;
long triggerSentTime = -1;
float distanceAverage = -1;
float alpha = 0.1;
float maxDiff = 300;

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
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, HIGH);
  enabled = true;
  currMotionState = MOTION_STATE_UP;
}

void setMoveDown() {
  if (enabled) { 
    return; 
  }
  Serial.println("setMoveDown");
  digitalWrite(ENABLE_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  enabled = true;
  currMotionState = MOTION_STATE_DOWN;
}

void resetAcceleration() {
  currSpeedDelayMs = maxSpeedDelayMs;
  prevLoopTimeMs = -1;
}

void setStop() {
  if (!enabled) { 
    return; 
  }
  Serial.println((String) "setStop at height: " + currDeskHeight);
  enabled = false;
  currMotionState = MOTION_STATE_DISABLED;
  resetAcceleration();
  storeDeskHeight(currDeskHeight);
  setDeskHeightBoundaries(0, MOTOR_MAX_STEPS);
  // Disable motor after a short delay to release torque more gracefully
  // The power consumption when motor is disabled is 6W and where it has holding torque it's 9.5W.
  // The savings are negligible and not holding the torque can lead to miscalibration of height over time.
  // delay(500);
  // digitalWrite(ENABLE_PIN, HIGH);
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
  int moveDownButtonCurrState = digitalRead(DESK_DOWN_PIN);
  if (moveUpButtonCurrState == LOW && moveUpButtonLastState == HIGH) {
    setMoveUp();
  } else if (moveUpButtonCurrState == HIGH && moveUpButtonLastState == LOW) {
    setStop();
  }

  if (moveDownButtonCurrState == LOW && moveDownButtonLastState == HIGH) {
    setMoveDown();
  } else if (moveDownButtonCurrState == HIGH && moveDownButtonLastState == LOW) {
    setStop();
  }
  moveUpButtonLastState = moveUpButtonCurrState;
  moveDownButtonLastState = moveDownButtonCurrState;
}

bool isWithinHeightBoundaries(int height) {
  return height >= minDeskHeight && height <= maxDeskHeight;
}

// IMPORTANT: Only use this method as a one-off when desk height is miscalibrated.
// After resetting the height, comment out all invocations and reupload the code.
void resetDeskHeightToZero() {
  storeDeskHeight(0);
}

void updateAverageDistance(float distance) {
  if (distanceAverage == -1) {
    distanceAverage = distance;
  } else {
    int diff = abs(distance - distanceAverage);
    if (diff <= maxDiff) {
      distanceAverage = alpha * distance + (1 - alpha) * distanceAverage;
    }
  }
}

void checkEchoPin() {
  if (digitalRead(DIST_SENSOR_ECHO_PIN) == LOW) {
    long duration = micros() - triggerSentTime;
    float distance = duration * 0.34 / 2;
    float oldDistanceAverage = distanceAverage;
    updateAverageDistance(distance);
    distanceTriggerState = DIST_IDLE;
  }
}

void sendDistanceTrigger() {
  digitalWrite(DIST_SENSOR_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(DIST_SENSOR_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(DIST_SENSOR_TRIG_PIN, LOW);
  distanceTriggerState = DIST_TRIGGER_SENT;
}

void setup() {
  Serial.begin(921600);
  pinMode(DESK_UP_PIN, INPUT_PULLUP);
  pinMode(DESK_DOWN_PIN, INPUT_PULLUP);
  pinMode(DESK_DOWN_GROUND_PIN, INPUT_PULLDOWN);
  pinMode(PRESET_1_PIN, INPUT_PULLUP);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(PRESET_2_PIN, INPUT_PULLUP);
  pinMode(DIST_SENSOR_ECHO_PIN, INPUT_PULLDOWN);
  pinMode(DIST_SENSOR_TRIG_PIN, OUTPUT);
  // resetDeskHeightToZero();
  currDeskHeight = readDeskHeight();
  Serial.println((String)"Initial desk height: " + currDeskHeight);
}

void loop() {
  checkPresetButtonStates();
  checkMoveButtonStates();
  if (distanceTriggerState == DIST_IDLE) {
    sendDistanceTrigger();
  } else if (distanceTriggerState == DIST_TRIGGER_SENT) {
    if (digitalRead(DIST_SENSOR_ECHO_PIN) == HIGH) {
      triggerSentTime = micros();
      distanceTriggerState = DIST_TIMER_STARTED;
    }
  } else if (distanceTriggerState == DIST_TIMER_STARTED) {
    checkEchoPin();
  }
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
    delayMicroseconds((int) (delayBasicMs * 1000)); // 100 microseconds
    digitalWrite(STEP_PIN, LOW);
    int t = micros();
    Serial.println(distanceAverage);
    int t2 = micros() - t;
    int delay = currSpeedDelayMs * 1000;
    if (t2 < delay) {
      delayMicroseconds(delay - t2);
    }

    // delayMicroseconds((int) (currSpeedDelayMs * 1000));
    currDeskHeight += currMotionState;
  } else if (enabled && !isWithinHeightBoundaries(currDeskHeight + currMotionState)) {
    setStop();
  } else {
  }
}