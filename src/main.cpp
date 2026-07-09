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
// Tune these to reduce rapid re-turning / allow time for sensing
const unsigned long turnCooldownMs = 120;      // minimum time between turns
const unsigned long minThinkTimeMs = 40;       // extra time before new turn decision


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

const uint8_t totalPages = 4;
const char* pageNames[totalPages] = {"Greetings", "Eyes", "Autonomous Drive"};
uint8_t currentPage = 0;

VL53L0X sensorLeft;
VL53L0X sensorRight;
float distanceFront = -1;
uint16_t distLeft = 65535;
uint16_t distRight = 65535;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;

// Function declarations
void beep(uint8_t times=1);
void startMelody();
void updateMelody();
void moveForward(uint8_t left, uint8_t right);
void moveBackward(uint8_t left, uint8_t right);
void turnRight(uint8_t left, uint8_t right);
void turnLeft(uint8_t left, uint8_t right);
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
void ultrasonicHybridDrive();
void sensorPowerDown(int pin);
void sensorPowerUp(int pin);
void initSensors();
void readUltrasonic();
void updateOLEDVL53(float front, uint16_t left, uint16_t right, const char* motorState);

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

  beep(2);
  currentPage=0;
  showWelcomePage();
}

void loop() {
  static uint8_t prevBtn1 = HIGH, prevBtn2 = HIGH;
  uint8_t btn1 = digitalRead(BTN1_PIN);
  uint8_t btn2 = digitalRead(BTN2_PIN);

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
      case 3: break;
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
}

void moveBackward(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
}

// Non-blocking turning state
volatile bool turning = false;
unsigned long turnStartMs = 0;
unsigned long lastTurnEndMs = 0;
unsigned long lastTurnCommandMs = 0;

void startTurnRight(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
  turning = true;
  turnStartMs = millis();
  lastTurnCommandMs = turnStartMs;
}

void startTurnLeft(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelA, left);
  ledcWrite(pwmChannelB, right);
  turning = true;
  turnStartMs = millis();
  lastTurnCommandMs = turnStartMs;
}

void updateTurning() {
  if (!turning) return;
  if (millis() - turnStartMs >= turn90Time) {
    stopMotors();
    turning = false;
    lastTurnEndMs = millis();
  }
}

// Backward-compatible wrappers (no longer used in autonomous logic)
void turnRight(uint8_t left, uint8_t right) {
  startTurnRight(left, right);
  updateTurning();
}

void turnLeft(uint8_t left, uint8_t right) {
  startTurnLeft(left, right);
  updateTurning();
}


void stopMotors() {
  ledcWrite(pwmChannelA, 0);
  ledcWrite(pwmChannelB, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);
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
  for(uint8_t i=0; i<totalPages; i++) {
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
  unsigned long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 30000);
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
  distLeft = sensorLeft.readRangeContinuousMillimeters();
  distRight = sensorRight.readRangeContinuousMillimeters();
  if(sensorLeft.timeoutOccurred()) distLeft = 65535;
  if(sensorRight.timeoutOccurred()) distRight = 65535;

  readUltrasonic();

  bool obstacleFront = (distanceFront > 0 && distanceFront < 15);
  bool obstacleLeft = (distLeft != 65535 && distLeft < 150);
  bool obstacleRight = (distRight != 65535 && distRight < 150);

  const int forwardSpeed = 150;
  const int turnSpeed = 120;
  const int backwardSpeed = 150;
  const char* motorState;

  if(obstacleFront && obstacleLeft && obstacleRight) {
    motorState = "Backward";
    moveBackward(backwardSpeed, backwardSpeed);
  } else if(obstacleLeft && !obstacleRight) {
    motorState = "Turn Right";
    turnRight(turnSpeed, turnSpeed);
  } else if(obstacleRight && !obstacleLeft) {
    motorState = "Turn Left";
    turnLeft(turnSpeed, turnSpeed);
  } else if(obstacleFront) {
    if(!obstacleLeft) {
      motorState = "Turn Left (free)";
      turnLeft(turnSpeed, turnSpeed);
    } else if(!obstacleRight) {
      motorState = "Turn Right (free)";
      turnRight(turnSpeed, turnSpeed);
    } else {
      motorState = "Stop";
      stopMotors();
    }
  } else {
    motorState = "Forward";
    moveForward(forwardSpeed, forwardSpeed);
  }

  updateOLEDVL53(distanceFront, distLeft, distRight, motorState);

  Serial.printf("Front: %.1f cm, Left: %d mm, Right: %d mm, Action: %s\n",
                distanceFront, distLeft, distRight, motorState);
}
