#include <Arduino.h>
#include <EEPROM.h>

#define DESK_UP_PIN 12
#define DESK_DOWN_PIN 14
#define DESK_DOWN_GROUND_PIN 26
#define PRESET_1_PIN 25
#define PRESET_2_PIN 32

#define DIR_PIN 19
#define DIR_GROUND_PIN 18
#define STEP_PIN 4
#define EEPROM_SIZE 3
#define EEPROM_MAX_VALUE 255
#define MOTION_STATE_UP 1
#define MOTION_STATE_DOWN -1
#define MOTION_STATE_DISABLED 0

const double delayBasicMs = 0.1;
int accelerationTimeMs = 1800;
double maxSpeedDelayMs = 1.5;
double minSpeedDelayMs = 0.23;
double currSpeedDelayMs = maxSpeedDelayMs;
double delayDeltaPerMs = (maxSpeedDelayMs - minSpeedDelayMs) / accelerationTimeMs;
int prevTimeMs = -1;

const int MOTOR_MAX_STEPS = 183000;
bool enabled = false;
int maxDeskHeightInSteps = MOTOR_MAX_STEPS;
int minDeskHeightInSteps = 0;
int currDeskHeightInSteps = 0;
int currMotionState = 0; // -1: moving down, 1: moving up, 0: not moving

int lastMoveUpButtonState = HIGH;
int lastMoveDownButtonState = HIGH;

const int PRESET_BUTTONS_PINS[] = {25, 32};
int presetButtonStates[] = {HIGH, HIGH};
int presetButtonLastState = HIGH;
int presetButtonPressedTimeMs = -1;

int toNormalized(int deskHeightInSteps) {
  return (double) deskHeightInSteps / MOTOR_MAX_STEPS * EEPROM_MAX_VALUE;
}

int fromNormalized(int deskHeightNormalized) {
  return (double) deskHeightNormalized / EEPROM_MAX_VALUE * MOTOR_MAX_STEPS;
}

void storePreset(int presetNumber, int deskHeightInSteps) {
  int deskHeightNormalized = toNormalized(deskHeightInSteps);
  Serial.print("Storing preset: ");
  Serial.print(presetNumber);
  Serial.print(", ");
  Serial.println(deskHeightNormalized);
  EEPROM.write(presetNumber, deskHeightNormalized);
  EEPROM.commit();
}

int readPreset(int presetNumber) {
  int normalizedDeskHeight = EEPROM.read(presetNumber);
  Serial.print("Reading preset: ");
  Serial.print(presetNumber);
  Serial.print(", ");
  Serial.println(normalizedDeskHeight);

  return fromNormalized(normalizedDeskHeight);
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
  Serial.println("setStop");
  enabled = false;
  currMotionState = MOTION_STATE_DISABLED;
  currSpeedDelayMs = maxSpeedDelayMs;
  prevTimeMs = -1;
  Serial.println(currDeskHeightInSteps);
  EEPROM.write(0, currDeskHeightInSteps / MOTOR_MAX_STEPS * EEPROM_MAX_VALUE);
  EEPROM.commit();
  maxDeskHeightInSteps = MOTOR_MAX_STEPS;
  minDeskHeightInSteps = 0;
}

void setMoveToPresetHeight(int heightInSteps) {
  if (enabled) {
    return;
  }
  Serial.print("preset: ");
  Serial.println(heightInSteps);
  if (currDeskHeightInSteps > heightInSteps) {
    minDeskHeightInSteps = max(0, heightInSteps);
    setMoveDown();
  } else {
    maxDeskHeightInSteps = min(MOTOR_MAX_STEPS, heightInSteps);
    setMoveUp();
  }
  Serial.print("Setting min max desk heights: ");
  Serial.print(minDeskHeightInSteps);
  Serial.print(", ");
  Serial.println(maxDeskHeightInSteps);
}

void onPresetButtonDown(int presetNumber) {
  if (presetButtonStates[presetNumber] == HIGH) {
    Serial.println("Preset button pressed");
    presetButtonPressedTimeMs = millis();
    presetButtonStates[presetNumber] = LOW; 
  }
}

void onPresetButtonUp(int presetNumber) {
  if (presetButtonStates[presetNumber] == LOW) {
    Serial.println("Preset button released");
    if (millis() - presetButtonPressedTimeMs > 2000) {
      storePreset(presetNumber + 1, currDeskHeightInSteps);
    } else {
      setMoveToPresetHeight(readPreset(presetNumber + 1));
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
  int currMoveUpButtonState = digitalRead(DESK_UP_PIN);
  if (currMoveUpButtonState == LOW && lastMoveUpButtonState == HIGH) {
    setMoveUp();
    lastMoveUpButtonState = LOW;
  } else if (currMoveUpButtonState == HIGH && lastMoveUpButtonState == LOW) {
    setStop();
    lastMoveUpButtonState = HIGH;
  }

  int currMoveDownButtonState = digitalRead(DESK_DOWN_PIN);
  if (currMoveDownButtonState == LOW && lastMoveDownButtonState == HIGH) {
    setMoveDown();
    lastMoveDownButtonState = LOW;
  } else if (currMoveDownButtonState == HIGH && lastMoveDownButtonState == LOW) {
    setStop();
    lastMoveDownButtonState = HIGH;
  }
}

bool isWithinHeightBoundaries(int height) {
  return height >= minDeskHeightInSteps && height <= maxDeskHeightInSteps;
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
  EEPROM.begin(EEPROM_SIZE);
  // EEPROM.write(0, 0);
  // EEPROM.commit();
  currDeskHeightInSteps = fromNormalized(EEPROM.read(0));
}

void loop() {
  checkPresetButtonStates();
  checkMoveButtonStates();
  if (enabled && isWithinHeightBoundaries(currDeskHeightInSteps + currMotionState)) {
    int currTimeMs = millis();
    if (prevTimeMs == -1) {
      prevTimeMs = currTimeMs;
    } else if (currSpeedDelayMs > minSpeedDelayMs) {
      int elapsedTimeMs = currTimeMs - prevTimeMs;
      currSpeedDelayMs -= delayDeltaPerMs * elapsedTimeMs;
      currSpeedDelayMs = max(currSpeedDelayMs, minSpeedDelayMs);
      prevTimeMs = currTimeMs;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds((int) (delayBasicMs * 1000));
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds((int) (currSpeedDelayMs * 1000));
    currDeskHeightInSteps += currMotionState;
  } else if (enabled && !isWithinHeightBoundaries(currDeskHeightInSteps + currMotionState)) {
    setStop();
  }
}