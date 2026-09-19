#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <VL53L0X.h>
#include <math.h>
#include "pitches.h"

// Pin definitions
#define AIN1 2
#define AIN2 3
#define PWMA 38
#define BIN1 14
#define BIN2 13
#define PWMB 39
#define STBY 1
#define BUZZER_PIN 8
#define BTN1_PIN 0
#define BTN2_PIN 9
#define SDA_PIN 21
#define SCL_PIN 20
#define NEOPIXEL_PIN 48
#define NEOPIXEL_NUM 1
#define ULTRASONIC_TRIG_PIN 19
#define ULTRASONIC_ECHO_PIN 4
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C
#define XSHUT_PIN1 16
#define XSHUT_PIN2 17
#define LOX1_ADDR 0x30
#define LOX2_ADDR 0x31

const int pwmChannelA = 0, pwmChannelB = 1, buzzerChannel = 2;
const int pwmFreq = 5000, pwmRes = 8;
const unsigned long turn90Time = 190;

int ref_eye_height = 40, ref_eye_width = 40, ref_space_between_eye = 10, ref_corner_radius = 10;
int baseLeftEyeX, baseLeftEyeY, baseRightEyeX, baseRightEyeY;
int left_eye_x, left_eye_y, right_eye_x, right_eye_y;
int left_eye_height, left_eye_width, right_eye_height, right_eye_width;
const int eyeOffsetMax = 14;
const float easingFactor = 0.12;
float smoothLeftX, smoothLeftY, smoothRightX, smoothRightY;
unsigned long lastEyeTime = 0;
int eyeState = 0;
const int eyeInterval = 1400;

int melody[] = {NOTE_E4, NOTE_FS4, NOTE_G4, 0, NOTE_G4, NOTE_B4, NOTE_A4, 0,
                NOTE_G4, NOTE_FS4, NOTE_D4, 0, NOTE_E4, NOTE_FS4, NOTE_G4, 0,
                NOTE_G4, NOTE_B4, NOTE_A4, 0, NOTE_D5, NOTE_B4, NOTE_A4, 0};
int noteDurations[] = {4,4,4,8,4,4,2,8,4,4,2,8,4,4,4,8,4,4,2,8,4,4,2,8};
bool melodyPlaying = true;
int melodyIndex = 0;
unsigned long noteStart = 0;
int noteLength = 0;

// Page 0 = Welcome, 1 = Greeting, 2 = Eyes, 3 = Autonomous Drive, 4 = Pose Telemetry
const uint8_t totalPages = 5;
const char* pageNames[totalPages - 1] = {"Greetings", "Eyes", "Autonomous Drive", "Pose Telemetry"};
uint8_t currentPage = 0;

VL53L0X sensorLeft;
VL53L0X sensorRight;
float distanceFront = -1;
uint16_t distLeft = 65535;
uint16_t distRight = 65535;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

// -------------------- Non-blocking turn state --------------------
enum TurnState { TURN_NONE, TURNING_LEFT, TURNING_RIGHT };
TurnState turnState = TURN_NONE;
unsigned long turnStartTime = 0;

// -------------------- Dead-reckoning pose state --------------------
// NOTE: with no wheel encoders, x/y are rough estimates based on an assumed
// constant speed at a given PWM value. Heading (from gyro) is far more
// trustworthy than position. Treat x/y as "roughly where I've gone", not GPS-grade.
struct RobotPose {
  float x = 0;        // mm, relative to power-on position
  float y = 0;        // mm
  float heading = 0;  // degrees, 0 = initial forward direction
};
RobotPose pose;

float gyroZBiasDegPerSec = 0;
unsigned long lastPoseMicros = 0;

// TUNE THESE: run moveForward(forwardSpeed,forwardSpeed) for exactly 2s on a
// known-length straight run, measure distance traveled in mm, divide by 2.
// Repeat for backward. These placeholders are almost certainly wrong for your robot.
const float FORWARD_SPEED_MM_S = 250.0;
const float BACKWARD_SPEED_MM_S = 250.0;

enum DriveState { DRIVE_STOPPED, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_TURNING };
DriveState driveState = DRIVE_STOPPED;

// Debounce: require N consecutive "obstacle" readings before acting on them.
// A single glitchy VL53L0X reading (crosstalk between the two side sensors,
// a stray reflection, etc.) used to be enough to trigger a full turn. This
// is the direct fix for spurious repeated turning with nothing actually there.
const int OBSTACLE_CONFIRM_COUNT = 3;
int leftObstacleStreak = 0;
int rightObstacleStreak = 0;
int frontObstacleStreak = 0;

// Function declarations
void beep(uint8_t times=1);
void startMelody();
void updateMelody();
void moveForward(uint8_t left, uint8_t right);
void moveBackward(uint8_t left, uint8_t right);
void turnRightStart(uint8_t left, uint8_t right);
void turnLeftStart(uint8_t left, uint8_t right);
bool turnUpdate();
void stopMotors();
void draw_eyes();
void center_eyes();
void blink(int speed=5);
void look_up();
void wink();
void updateEyeAnimation();
void updateEyesWithMPU();
void showWelcomePage();
void showGreetingPage();
void showPosePage();
void ultrasonicHybridDrive();
void sensorPowerDown(int pin);
void sensorPowerUp(int pin);
void initSensors();
void readUltrasonic();
void updateOLEDVL53(float front, uint16_t left, uint16_t right, const char* motorState);
void calibrateGyroBias();
void updatePose();
void predictPosition(float secondsAhead, float &predX, float &predY);

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(BTN1_PIN, INPUT_PULLUP);
  pinMode(BTN2_PIN, INPUT_PULLUP);
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  ledcSetup(pwmChannelA, pwmFreq, pwmRes);
  ledcSetup(pwmChannelB, pwmFreq, pwmRes);
  ledcSetup(buzzerChannel, 3000, pwmRes);
  ledcAttachPin(BUZZER_PIN, buzzerChannel);
  ledcAttachPin(PWMA, pwmChannelA);
  ledcAttachPin(PWMB, pwmChannelB);

  Wire.begin(SDA_PIN, SCL_PIN);
  if(!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
    while(1);
  }
  display.clearDisplay();

  pixels.begin();
  pixels.setBrightness(90);
  pixels.setPixelColor(0, pixels.Color(128,0,128));
  pixels.show();

  if(!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while(1);
  }

  baseLeftEyeX = SCREEN_WIDTH/2 - ref_eye_width/2 - ref_space_between_eye/2;
  baseLeftEyeY = SCREEN_HEIGHT/2;
  baseRightEyeX = SCREEN_WIDTH/2 + ref_eye_width/2 + ref_space_between_eye/2;
  baseRightEyeY = SCREEN_HEIGHT/2;

  left_eye_x = baseLeftEyeX; left_eye_y = baseLeftEyeY;
  right_eye_x = baseRightEyeX; right_eye_y = baseRightEyeY;

  left_eye_height = right_eye_height = ref_eye_height;
  left_eye_width = right_eye_width = ref_eye_width;

  smoothLeftX = left_eye_x;
  smoothLeftY = left_eye_y;
  smoothRightX = right_eye_x;
  smoothRightY = right_eye_y;

  initSensors();

  // Keep the robot still while this runs — takes ~1 second
  calibrateGyroBias();
  lastPoseMicros = micros();

  beep(2);
  currentPage=0;
  showWelcomePage();
}

void loop() {
  static uint8_t prevBtn1 = HIGH, prevBtn2 = HIGH;
  uint8_t btn1 = digitalRead(BTN1_PIN);
  uint8_t btn2 = digitalRead(BTN2_PIN);

  // Pose integration runs every cycle regardless of page, so heading
  // doesn't get stale gaps when you're on the Eyes or Greeting page.
  updatePose();

  if(btn1 == LOW && prevBtn1 == HIGH) {
    beep(1);
    currentPage = (currentPage + 1) % totalPages;
    display.clearDisplay();

    switch(currentPage) {
      case 0: showWelcomePage(); break;
      case 1: showGreetingPage(); break;
      case 2:
        center_eyes();
        draw_eyes();
        display.display();
        break;
      case 3: break; // Autonomous Drive renders itself continuously below
      case 4: showPosePage(); break;
    }
    delay(250);
  }

  if(btn2 == LOW && prevBtn2 == HIGH) {
    beep(1);
    delay(200);
  }
  prevBtn1 = btn1;
  prevBtn2 = btn2;

  if(currentPage == 1) {
    stopMotors();
  } else if(currentPage == 2) {
    updateEyesWithMPU();
    updateEyeAnimation();
    display.clearDisplay();
    draw_eyes();
    display.display();
    stopMotors();
  } else if(currentPage == 3) {
    ultrasonicHybridDrive();
  } else if(currentPage == 4) {
    showPosePage();
  } else {
    stopMotors();
  }
}

// -------------------- Eyes functions --------------------
void draw_eyes() {
  display.fillRoundRect(left_eye_x - left_eye_width / 2, left_eye_y - left_eye_height / 2,
                        left_eye_width, left_eye_height, ref_corner_radius, SSD1306_WHITE);
  display.fillRoundRect(right_eye_x - right_eye_width / 2, right_eye_y - right_eye_height / 2,
                        right_eye_width, right_eye_height, ref_corner_radius, SSD1306_WHITE);
}

void center_eyes() {
  left_eye_x = baseLeftEyeX;
  left_eye_y = baseLeftEyeY;
  right_eye_x = baseRightEyeX;
  right_eye_y = baseRightEyeY;
  left_eye_height = right_eye_height = ref_eye_height;
  left_eye_width = right_eye_width = ref_eye_width;
}

void blink(int speed) {
  int lh = ref_eye_height;
  for (int i = 0; i < ref_eye_height / 2; i += speed) {
    left_eye_height = right_eye_height = ref_eye_height - i;
    display.clearDisplay();
    draw_eyes();
    display.display();
    delay(24);
  }
  for (int i = 0; i < ref_eye_height / 2; i += speed) {
    left_eye_height = right_eye_height = lh - ref_eye_height / 2 + i;
    display.clearDisplay();
    draw_eyes();
    display.display();
    delay(24);
  }
  left_eye_height = right_eye_height = ref_eye_height;
}

void look_up() {
  left_eye_y = baseLeftEyeY - 5;
  right_eye_y = baseRightEyeY - 5;
}

void wink() {
  center_eyes();
  left_eye_height = 2;
  display.clearDisplay();
  draw_eyes();
  display.display();
  delay(500);
  left_eye_height = ref_eye_height;
}

void updateEyeAnimation() {
  if (millis() - lastEyeTime > eyeInterval) {
    lastEyeTime = millis();
    int action = random(0,4); // 0=blink,1=look up,2=wink,3=subtle idle
    switch(action) {
      case 0: blink(3); break;
      case 1: look_up(); break;
      case 2: wink(); break;
      case 3:
        left_eye_x += random(-2,3);
        left_eye_y += random(-2,3);
        right_eye_x += random(-2,3);
        right_eye_y += random(-2,3);
        break;
    }
  }
}

void updateEyesWithMPU() {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float ax = a.acceleration.x, ay = a.acceleration.y, az = a.acceleration.z;
  float roll = atan2(ay, az) * 180.0 / PI;
  float pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI;
  int targetX = map((int)pitch, -30, 30, -eyeOffsetMax, eyeOffsetMax);
  int targetY = map((int)roll, -30, 30, -eyeOffsetMax, eyeOffsetMax);
  smoothLeftX += (baseLeftEyeX + targetX - smoothLeftX) * easingFactor;
  smoothLeftY += (baseLeftEyeY + targetY - smoothLeftY) * easingFactor;
  smoothRightX += (baseRightEyeX + targetX - smoothRightX) * easingFactor;
  smoothRightY += (baseRightEyeY + targetY - smoothRightY) * easingFactor;
  left_eye_x = (int)smoothLeftX;
  left_eye_y = (int)smoothLeftY;
  right_eye_x = (int)smoothRightX;
  right_eye_y = (int)smoothRightY;
}

// -------------------- Motors --------------------
void moveForward(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
  driveState = DRIVE_FORWARD;
}

void moveBackward(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
  driveState = DRIVE_BACKWARD;
}

// Starts a turn WITHOUT blocking. Call turnUpdate() every loop to finish it.
void turnRightStart(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
  turnState = TURNING_RIGHT;
  turnStartTime = millis();
  driveState = DRIVE_TURNING;
}

void turnLeftStart(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
  turnState = TURNING_LEFT;
  turnStartTime = millis();
  driveState = DRIVE_TURNING;
}

// Non-blocking: returns true while a turn is still in progress.
// Call this at the top of any drive loop; if it returns true, skip
// re-reading sensors / re-deciding this cycle.
bool turnUpdate() {
  if (turnState == TURN_NONE) return false;
  if (millis() - turnStartTime >= turn90Time) {
    stopMotors();
    turnState = TURN_NONE;
    return false;
  }
  return true;
}

void stopMotors() {
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);
  driveState = DRIVE_STOPPED;
  turnState = TURN_NONE;
}

// -------------------- Display --------------------
void showWelcomePage() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  String robotName = "MAZERUNNER";
  int16_t xb,yb; uint16_t wb,hb;
  display.getTextBounds(robotName, 0,0, &xb,&yb,&wb,&hb);
  display.setCursor((SCREEN_WIDTH - wb) / 2, 7);
  display.println(robotName);

  display.setTextSize(1);
  display.setCursor(0, 28);
  display.println("Menu Pages:");
  for(uint8_t i=0; i<totalPages-1; i++) {
    display.setCursor(0, 38 + i*9);
    display.print(i+1);
    display.print(". ");
    display.println(pageNames[i]);
  }
  display.display();
}

void showGreetingPage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(20, 10);
  display.println("Heyya");
  display.setCursor(20, 30);
  display.println("User!");
  display.setCursor(90, 30);
  display.println("<3");
  display.display();
}

void showPosePage() {
  float predX, predY;
  predictPosition(1.0f, predX, predY);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("X:   %.0f mm\n", pose.x);
  display.printf("Y:   %.0f mm\n", pose.y);
  display.printf("Hdg: %.1f deg\n", pose.heading);
  display.println();
  display.println("Predicted +1s:");
  display.printf("(%.0f, %.0f)\n", predX, predY);
  display.display();
}

// -------------------- Sound --------------------
void beep(uint8_t times) {
  for (uint8_t i = 0; i < times; i++) {
    ledcWriteTone(buzzerChannel, 3000);
    delay(120);
    ledcWriteTone(buzzerChannel, 0);
    delay(120);
  }
}

void startMelody() {
  melodyPlaying = true;
  melodyIndex = 0;
  noteLength = 1000 / noteDurations[melodyIndex];
  noteStart = millis();
  ledcWriteTone(buzzerChannel, melody[melodyIndex]);
}

void updateMelody() {
  if (!melodyPlaying) return;
  if (millis() - noteStart >= (unsigned long)noteLength) {
    melodyIndex++;
    if (melodyIndex >= (int)(sizeof(melody)/sizeof(int))) {
      melodyPlaying = false;
      ledcWriteTone(buzzerChannel, 0);
    } else {
      noteLength = 1000 / noteDurations[melodyIndex];
      noteStart = millis();
      ledcWriteTone(buzzerChannel, melody[melodyIndex]);
    }
  }
}

// -------------------- Sensors --------------------
void sensorPowerDown(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void sensorPowerUp(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH);
  delay(10);
}

void initSensors() {
  sensorPowerDown(XSHUT_PIN1);
  sensorPowerDown(XSHUT_PIN2);
  delay(10);

  sensorPowerUp(XSHUT_PIN1);
  if (!sensorLeft.init()) {
    Serial.println("Left VL53 init failed");
    while(1);
  }
  sensorLeft.setAddress(LOX1_ADDR);

  sensorPowerUp(XSHUT_PIN2);
  if (!sensorRight.init()) {
    Serial.println("Right VL53 init failed");
    while(1);
  }
  sensorRight.setAddress(LOX2_ADDR);

  sensorLeft.startContinuous();
  sensorRight.startContinuous();
}

void readUltrasonic() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  // Timeout reduced from 30000us to 8000us (~1.7m range, plenty for obstacle
  // avoidance) so a "nothing in range" reading doesn't stall the loop for 30ms.
  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 8000);
  distanceFront = (duration > 0) ? (duration / 2.0) / 29.1 : -1;
}

void updateOLEDVL53(float front, uint16_t left, uint16_t right, const char* motorState) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.printf("Front: %s\n", (front > 0) ? String(front, 1).c_str() : "Out");
  display.printf("Left : %s\n", (left != 65535) ? String(left).c_str() : "Out");
  display.printf("Right: %s\n", (right != 65535) ? String(right).c_str() : "Out");
  display.printf("Motor: %s", motorState);
  display.display();
}

void ultrasonicHybridDrive() {
  // If mid-turn, let it finish before reading sensors / making a new decision.
  // This is the fix for the "lag then sudden turn then stop" behavior — the
  // old code used delay(190) inside turnRight/turnLeft which froze everything
  // (sensors, OLED, buttons) for the full turn duration.
  if (turnUpdate()) {
    return;
  }

  distLeft = sensorLeft.readRangeContinuousMillimeters();
  distRight = sensorRight.readRangeContinuousMillimeters();
  if(sensorLeft.timeoutOccurred()) distLeft = 65535;
  if(sensorRight.timeoutOccurred()) distRight = 65535;

  readUltrasonic();

  bool rawFront = (distanceFront > 0 && distanceFront < 15);
  bool rawLeft  = (distLeft != 65535 && distLeft < 150);
  bool rawRight = (distRight != 65535 && distRight < 150);

  frontObstacleStreak = rawFront ? frontObstacleStreak + 1 : 0;
  leftObstacleStreak  = rawLeft  ? leftObstacleStreak + 1  : 0;
  rightObstacleStreak = rawRight ? rightObstacleStreak + 1 : 0;

  bool obstacleFront = (frontObstacleStreak >= OBSTACLE_CONFIRM_COUNT);
  bool obstacleLeft  = (leftObstacleStreak  >= OBSTACLE_CONFIRM_COUNT);
  bool obstacleRight = (rightObstacleStreak >= OBSTACLE_CONFIRM_COUNT);

  const int forwardSpeed = 150;
  const int turnSpeed = 60;
  const int backwardSpeed = 150;
  const char* motorState;

  if(obstacleFront && obstacleLeft && obstacleRight) {
    motorState = "Backward";
    moveBackward(backwardSpeed, backwardSpeed);
  } else if(obstacleLeft && !obstacleRight) {
    motorState = "Turn Right";
    turnRightStart(turnSpeed, turnSpeed);
  } else if(obstacleRight && !obstacleLeft) {
    motorState = "Turn Left";
    turnLeftStart(turnSpeed, turnSpeed);
  } else if(obstacleFront) {
    if(!obstacleLeft) {
      motorState = "Turn Left (free)";
      turnLeftStart(turnSpeed, turnSpeed);
    } else if(!obstacleRight) {
      motorState = "Turn Right (free)";
      turnRightStart(turnSpeed, turnSpeed);
    } else {
      motorState = "Stop";
      stopMotors();
    }
  } else {
    motorState = "Forward";
    moveForward(forwardSpeed, forwardSpeed);
  }

  updateOLEDVL53(distanceFront, distLeft, distRight, motorState);

  Serial.printf("Front: %.1f cm (streak %d), Left: %d mm (streak %d), Right: %d mm (streak %d), Action: %s\n",
                distanceFront, frontObstacleStreak, distLeft, leftObstacleStreak,
                distRight, rightObstacleStreak, motorState);
}

// -------------------- Pose Estimation (Dead Reckoning) --------------------
void calibrateGyroBias() {
  Serial.println("Calibrating gyro bias, keep robot still...");
  const int samples = 200;
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, t;
    mpu.getEvent(&a, &g, &t);
    sum += g.gyro.z * 180.0 / PI; // deg/s
    delay(5);
  }
  gyroZBiasDegPerSec = sum / samples;
  Serial.printf("Gyro Z bias: %.3f deg/s\n", gyroZBiasDegPerSec);
}

// Integrates gyro yaw rate into heading, and (very roughly) advances x/y
// based on which drive state is active and the calibrated speed constants.
// Call every loop() iteration, unconditionally.
void updatePose() {
  unsigned long nowMicros = micros();
  if (lastPoseMicros == 0) { lastPoseMicros = nowMicros; return; }
  float dt = (nowMicros - lastPoseMicros) / 1000000.0f; // seconds
  lastPoseMicros = nowMicros;
  if (dt <= 0 || dt > 1.0f) return; // guard against overflow/first-run glitches

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float yawRateDegPerSec = (g.gyro.z * 180.0 / PI) - gyroZBiasDegPerSec;
  pose.heading += yawRateDegPerSec * dt;
  if (pose.heading > 180) pose.heading -= 360;
  if (pose.heading < -180) pose.heading += 360;

  float speed = 0;
  if (driveState == DRIVE_FORWARD) speed = FORWARD_SPEED_MM_S;
  else if (driveState == DRIVE_BACKWARD) speed = -BACKWARD_SPEED_MM_S;
  // DRIVE_TURNING and DRIVE_STOPPED contribute no translation.

  if (speed != 0) {
    float headingRad = pose.heading * PI / 180.0f;
    pose.x += speed * dt * cos(headingRad);
    pose.y += speed * dt * sin(headingRad);
  }
}

// Simple constant-velocity extrapolation from current state — not a trained
// model, just physics. See chat notes on why this is the right first step
// before a neural net.
void predictPosition(float secondsAhead, float &predX, float &predY) {
  float speed = 0;
  if (driveState == DRIVE_FORWARD) speed = FORWARD_SPEED_MM_S;
  else if (driveState == DRIVE_BACKWARD) speed = -BACKWARD_SPEED_MM_S;
  float headingRad = pose.heading * PI / 180.0f;
  predX = pose.x + speed * secondsAhead * cos(headingRad);
  predY = pose.y + speed * secondsAhead * sin(headingRad);
}
