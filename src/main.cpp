#include <Arduino.h>
#include <Preferences.h>
#include "FastAccelStepper.h"
#include <TM1637Display.h>
#include <WiFi.h>
#include <HTTPClient.h>

// --- WiFi / Tasmota ---
// WIFI_SSID, WIFI_PASS, TASMOTA_IP are injected via build_flags from secrets.ini (see platformio.ini)
#define PSU_TIMEOUT_MS          15000UL          // 15 seconds

// --- Pin Definitions ---

// Display (TM1637, Side A)
#define DISPLAY_CLK_PIN         2
#define DISPLAY_DIO_PIN         15

// Buttons (Side A)
#define DESK_UP_PIN             16
#define DESK_DOWN_PIN           23
#define PRESET_1_PIN            17
#define PRESET_2_PIN            5
#define MODE_BUTTON_GND_PIN     22  // software GND (driven LOW)
#define MODE_BUTTON_PIN         18

// Motor driver (DM542Y, Side B)
#define STEPPER_GND_PIN         26  // software GND (driven LOW)
#define DIR_PIN                 25
#define STEP_PIN                33
#define ENABLE_PIN              32  // MF+ on DM542Y

// --- Motor Parameters ---
// Driver: DM542Y at 1600 steps/rev (1/8 microstepping, SW5-SW8 DIP switches).
// Motor:  1.8°/step = 200 full steps/rev × 8 = 1600 steps/rev.
//
// Speed formula: MOTOR_MAX_SPEED_HZ / 1600 steps/rev / 3 rev/cm
//   e.g. 4347 steps/s → ~0.91 cm/s → ~33 s for a 30 cm range (conservative)
//
// TODO: raise MOTOR_MAX_SPEED_HZ to ~14000 for a more typical desk speed of ~3 cm/s (~10 s for 30 cm).
#define MOTOR_MAX_SPEED_HZ      2500
#define MOTOR_ACCELERATION      500
#define STEPS_PER_CM            1338  // calibrated: 149800 steps = 112cm
#define DISPLAY_TIMEOUT_MS      10000

// Calibration mode: very slow movement so the desk can be inched to true zero.
#define CAL_SPEED_HZ            300
#define CAL_ACCELERATION        5000
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
TM1637Display display(DISPLAY_CLK_PIN, DISPLAY_DIO_PIN);

MoveDirection moveDirection = IDLE;
bool wasRunning = false;
int32_t lastDisplayedHeight = -1;
unsigned long lastActivityMs = 0;
bool displayOn = true;

int moveUpButtonLastState = HIGH;
int moveDownButtonLastState = HIGH;
const int PRESET_BUTTONS_PINS[] = {PRESET_2_PIN, PRESET_1_PIN};
int presetButtonStates[] = {HIGH, HIGH};
unsigned long presetButtonPressedTimeMs[] = {0, 0};

bool psuOn = true;

bool calibrationMode = false;
int modeButtonLastState = HIGH;
unsigned long modeButtonPressedTimeMs = 0;
unsigned long calBlinkLastMs = 0;
bool calBlinkOn = true;

// "CAL " — C=0x39, A=0x77, L=0x38, blank=0x00
const uint8_t CAL_SEGMENTS[]  = {0x39, 0x77, 0x38, 0x00};
const uint8_t BLANK_SEGMENTS[] = {0x00, 0x00, 0x00, 0x00};
// "----" shown while PSU is waking up
const uint8_t WAKE_SEGMENTS[]  = {0x40, 0x40, 0x40, 0x40};
// "Err " — E=0x79, r=0x50, r=0x50, blank
const uint8_t ERR_SEGMENTS[]   = {0x79, 0x50, 0x50, 0x00};
// "conn" — C=0x39, o=0x5C, n=0x54, n=0x54
const uint8_t CONN_SEGMENTS[]  = {0x39, 0x5C, 0x54, 0x54};

// --- WiFi / PSU ---

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\nWiFi connected: " + WiFi.localIP().toString());
  else
    Serial.println("\nWiFi connection failed");
}

// Returns true if the plug acknowledged the command.
bool setPsuPower(bool on) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http;
  String url = String("http://") + TASMOTA_IP + "/cm?cmnd=Power%20" + (on ? "On" : "Off");
  http.begin(url);
  http.setTimeout(3000);
  int code = http.GET();
  bool ok = false;
  if (code == 200) {
    String body = http.getString();
    ok = body.indexOf(on ? "\"ON\"" : "\"OFF\"") >= 0;
  }
  http.end();
  Serial.println(String("PSU ") + (on ? "on" : "off") + (ok ? " OK" : " FAILED"));
  return ok;
}

// Shows "----" while waiting for the PSU to come up.
// Returns false if the plug could not be reached.
bool wakeUpPsu() {
  display.setBrightness(0x0f, true);
  display.setSegments(WAKE_SEGMENTS);
  bool ok = setPsuPower(true);
  if (!ok) {
    display.setSegments(ERR_SEGMENTS);
    delay(1500);
    lastDisplayedHeight = -1;
    return false;
  }
  delay(800); // wait for PSU rails to stabilize
  psuOn = true;
  displayOn = true;
  lastActivityMs = millis();
  lastDisplayedHeight = -1;
  return true;
}

// --- NVS Helpers ---
void storePreset(unsigned int presetNumber, int32_t height) {
  Serial.println((String)"Storing preset " + presetNumber + " = " + height);
  char key[3];
  sprintf(key, "%u", presetNumber);
  preferences.begin(PRESET_PREFS_NAMESPACE);
  preferences.putUInt(key, (unsigned int)height);
  preferences.end();
}

int32_t readPreset(unsigned int presetNumber) {
  char key[3];
  sprintf(key, "%u", presetNumber);
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

// --- Display Helpers ---

void showHeight(int32_t steps) {
  display.showNumberDec(steps / STEPS_PER_CM, false);
}

void blinkDisplay(int32_t steps, int count) {
  for (int i = 0; i < count; i++) {
    display.setBrightness(0x0f, false);
    showHeight(steps);
    delay(100);
    display.setBrightness(0x0f, true);
    showHeight(steps);
    delay(100);
  }
}

// --- Motor Helpers ---
void setMotorSpeed(uint32_t hz, uint32_t accel) {
  stepper->setSpeedInHz(hz);
  stepper->setAcceleration(accel);
}

// --- Calibration ---
void enterCalibrationMode() {
  calibrationMode = true;
  modeButtonPressedTimeMs = 0; // mark long-press as consumed so the upcoming release is ignored
  Serial.println("Entering calibration mode");
  stepper->setCurrentPosition(MOTOR_MAX_STEPS / 2);
  setMotorSpeed(CAL_SPEED_HZ, CAL_ACCELERATION);
  displayOn = true;
  display.setBrightness(0x0f, true);
}

void exitCalibrationMode() {
  calibrationMode = false;
  stepper->forceStop();
  Serial.println("Exiting calibration mode, setting position to zero");
  stepper->setCurrentPosition(0);
  storeDeskHeight(0);
  moveDirection = IDLE;
  setMotorSpeed(MOTOR_MAX_SPEED_HZ, MOTOR_ACCELERATION);
  lastDisplayedHeight = -1; // force display refresh on next loop tick
  display.setBrightness(0x0f, true);
  showHeight(0);
}

void checkModeButtonState() {
  int state = digitalRead(MODE_BUTTON_PIN);
  if (state == LOW && modeButtonLastState == HIGH) {
    modeButtonPressedTimeMs = millis();
  } else if (state == LOW && !calibrationMode && modeButtonPressedTimeMs > 0 && millis() - modeButtonPressedTimeMs > 2000) {
    enterCalibrationMode();
  } else if (state == HIGH && modeButtonLastState == LOW) {
    if (calibrationMode && modeButtonPressedTimeMs > 0) {
      exitCalibrationMode();
    }
    modeButtonPressedTimeMs = 0;
  }
  modeButtonLastState = state;
}

void updateCalibrationDisplay() {
  unsigned long now = millis();
  if (now - calBlinkLastMs < 500) return;
  calBlinkLastMs = now;
  calBlinkOn = !calBlinkOn;
  display.setSegments(calBlinkOn ? CAL_SEGMENTS : BLANK_SEGMENTS);
}

// --- Motion ---

// Applies a short reverse (backtrack) to release leadscrew tension, then persists height.
void onMotorStop() {
  if (calibrationMode) {
    moveDirection = IDLE;
    return;
  }

  int backtrackSteps = (moveDirection == UP) ? -BACKTRACK_STEPS_UP : BACKTRACK_STEPS_DOWN;
  Serial.println((String)"Backtracking " + backtrackSteps + " steps");
  stepper->move(backtrackSteps, /*blocking=*/true);
  int32_t finalHeight = stepper->getCurrentPosition();
  Serial.println((String)"Stopped at height: " + finalHeight);
  storeDeskHeight(finalHeight);
  moveDirection = IDLE;

  blinkDisplay(finalHeight, 1);
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
      presetButtonPressedTimeMs[i] = millis();
      presetButtonStates[i] = LOW;
    } else if (state == LOW && presetButtonStates[i] == LOW) {
      if (presetButtonPressedTimeMs[i] > 0 && millis() - presetButtonPressedTimeMs[i] > 2000) {
        int32_t height = stepper->getCurrentPosition();
        storePreset(i + 1, height);
        blinkDisplay(height, 3);
        presetButtonPressedTimeMs[i] = 0;
      }
    } else if (state == HIGH && presetButtonStates[i] == LOW) {
      if (presetButtonPressedTimeMs[i] > 0) {
        if (psuOn || wakeUpPsu()) moveToHeight(readPreset(i + 1));
      }
      presetButtonPressedTimeMs[i] = 0;
      presetButtonStates[i] = HIGH;
    }
  }
}

void checkMoveButtonStates() {
  int upState   = digitalRead(DESK_UP_PIN);
  int downState = digitalRead(DESK_DOWN_PIN);

  if (upState == LOW && moveUpButtonLastState == HIGH) {
    if (psuOn || wakeUpPsu()) moveUp();
  } else if (upState == HIGH && moveUpButtonLastState == LOW) stepper->stopMove();

  if (downState == LOW && moveDownButtonLastState == HIGH) {
    if (psuOn || wakeUpPsu()) moveDown();
  } else if (downState == HIGH && moveDownButtonLastState == LOW) stepper->stopMove();

  moveUpButtonLastState   = upState;
  moveDownButtonLastState = downState;
}

// --- Debug ---
void debugMotorTest() {
  setMotorSpeed(500, 200);
  Serial.println("DEBUG: moving forward 1000 steps...");
  stepper->move(1000);
  while (stepper->isRunning()) delay(10);
  delay(500);
  Serial.println("DEBUG: moving backward 1000 steps...");
  stepper->move(-1000);
  while (stepper->isRunning()) delay(10);
  Serial.println("DEBUG: motor test done");
  setMotorSpeed(MOTOR_MAX_SPEED_HZ, MOTOR_ACCELERATION);
}

// --- Arduino Entry Points ---
void setup() {
  Serial.begin(921600);

  display.setBrightness(0x0f);
  display.setSegments(CONN_SEGMENTS);

  connectWiFi();
  setPsuPower(true);

  showHeight(0);

  pinMode(STEPPER_GND_PIN,       OUTPUT);
  digitalWrite(STEPPER_GND_PIN,  LOW);
  pinMode(DESK_UP_PIN,           INPUT_PULLUP);
  pinMode(DESK_DOWN_PIN,         INPUT_PULLUP);
  pinMode(PRESET_1_PIN,          INPUT_PULLUP);
  pinMode(PRESET_2_PIN,          INPUT_PULLUP);
  pinMode(MODE_BUTTON_GND_PIN,   OUTPUT);
  digitalWrite(MODE_BUTTON_GND_PIN, LOW);
  pinMode(MODE_BUTTON_PIN,       INPUT_PULLUP);

  engine.init();
  stepper = engine.stepperConnectToPin(STEP_PIN);
  stepper->setDirectionPin(DIR_PIN);
  stepper->setEnablePin(ENABLE_PIN);
  stepper->setAutoEnable(true);
  // Motor power consumption: 6W disabled, 9.5W with holding torque.
  // Disable after 500ms to release torque; savings are minor but avoids long-term miscalibration.
  stepper->setDelayToDisable(500);
  setMotorSpeed(MOTOR_MAX_SPEED_HZ, MOTOR_ACCELERATION);

  // debugMotorTest();
  // storeDeskHeight(0); // one-off recalibration: uncomment, upload, re-comment, re-upload
  lastActivityMs = millis();
  int32_t savedHeight = readDeskHeight();
  stepper->setCurrentPosition(savedHeight);
  Serial.println((String)"Initial desk height: " + savedHeight);
}

void loop() {
  checkModeButtonState();
  checkPresetButtonStates();
  checkMoveButtonStates();

  bool isRunning = stepper->isRunning();
  if (wasRunning && !isRunning) {
    onMotorStop();
  }
  wasRunning = isRunning;

  if (isRunning) lastActivityMs = millis();

  if (psuOn && !calibrationMode && !isRunning && millis() - lastActivityMs > PSU_TIMEOUT_MS) {
    if (WiFi.status() != WL_CONNECTED || setPsuPower(false)) psuOn = false;
  }

  if (calibrationMode) {
    updateCalibrationDisplay();
    return;
  }

  if (!displayOn && isRunning) {
    displayOn = true;
    display.setBrightness(0x0f, true);
    lastDisplayedHeight = -1;
  } else if (displayOn && !isRunning && millis() - lastActivityMs > DISPLAY_TIMEOUT_MS) {
    displayOn = false;
    display.setBrightness(0x0f, false);
    display.setSegments(BLANK_SEGMENTS); // push off state to hardware; setBrightness alone doesn't transmit
  }

  if (displayOn) {
    int32_t currentHeight = stepper->getCurrentPosition();
    if (currentHeight != lastDisplayedHeight) {
      showHeight(currentHeight);
      lastDisplayedHeight = currentHeight;
    }
  }
}
