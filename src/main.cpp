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
#include "nn_policy.h"
#include <WiFi.h>
#include <WebServer.h>
#include <driver/i2s.h>

// Local creds live in include/wifi_config.h (untracked). Empty = AP only.
#if __has_include("wifi_config.h")
#include "wifi_config.h"
#else
#define WIFI_SSID ""
#define WIFI_PASS ""
#define AP_SSID "MazeRunner"
#define AP_PASS "mazerunner123"
#endif

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

// INMP441 mic (same wiring as the MIC test board): VDD->3V3, GND->GND,
// SD->12, WS->11, SCK->10, L/R->GND. 10/11/12 are free on this bot.
#define I2S_WS 11
#define I2S_SCK 10
#define I2S_SD 12
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE 16000
#define I2S_READ_SAMPLES 128

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

// Page 0 = Welcome, 1 = Greeting, 2 = Eyes, 3 = Autonomous Drive,
// 4 = Pose Telemetry, 5 = Neural Drive, 6 = Voice
const uint8_t totalPages = 7;
const char* pageNames[totalPages - 1] = {"Greet", "Eyes", "Drive", "Pose", "Neural", "Voice"};
const uint8_t NEURAL_PAGE = 5;
const uint8_t VOICE_PAGE = 6;
uint8_t currentPage = 0;

VL53L0X sensorLeft;
VL53L0X sensorRight;
float distanceFront = -1;
uint16_t distLeft = 65535;
uint16_t distRight = 65535;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
WebServer webServer(80);

// -------------------- Web control state --------------------
int webSpeed = 150; // 0..255, set from the site slider
String lastVoiceText = ""; // last YOU/AI text from the PC script, shown on site + voice page
String botIP = "no wifi";

// -------------------- Voice page state --------------------
static bool micOk = false; // I2S mic installed? (lazy: only while on voice page)
static int16_t *voiceBuf = nullptr;
static size_t voiceCap = 0; // samples
static size_t voiceLen = 0;
static bool voiceRec = false;
static unsigned long voiceRecStart = 0;
// PSRAM (if the board has it) allows long holds, otherwise cap at 8s (256KB).
static size_t voiceMaxSec() { return psramFound() ? 30 : 8; }
enum VoiceStatus { V_IDLE, V_RECORDING, V_WAITING, V_RESULT, V_NOLINK };
VoiceStatus voiceStatus = V_IDLE;
unsigned long voiceWaitStart = 0;
String voiceHeard = ""; // what Vosk heard
String voiceReply = ""; // what ollama answered

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
float gyroYawRate = 0; // deg/s, filtered gyro z, used by the neural page
unsigned long lastPoseMicros = 0;

// TUNE THESE: run moveForward(forwardSpeed,forwardSpeed) for exactly 2s on a
// known-length straight run, measure distance traveled in mm, divide by 2.
// Repeat for backward. These placeholders are almost certainly wrong for your robot.
const float FORWARD_SPEED_MM_S = 250.0;
const float BACKWARD_SPEED_MM_S = 250.0;

enum DriveState { DRIVE_STOPPED, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_TURNING };
DriveState driveState = DRIVE_STOPPED;

// Neural page state
static float nnOutL = 0, nnOutR = 0;
static float nnPrevL = 800, nnPrevR = 800;

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
void showNeuralPage();
void neuralDrive();
void setWheels(int left, int right);
void ultrasonicHybridDrive();
void sensorPowerDown(int pin);
void sensorPowerUp(int pin);
void initSensors();
void readUltrasonic();
void updateOLEDVL53(float front, uint16_t left, uint16_t right, const char* motorState);
void calibrateGyroBias();
void updatePose();
void predictPosition(float secondsAhead, float &predX, float &predY);
void showVoicePage();
void setupWifi();
void setupWeb();
void handleWebRoot();
void handleWebCmd();
void handleWebSpeed();
void handleWebSensors();
void webBurst(const String& dir);
bool initMic();
void ensureMic();
void deinitMic();
void advancePage();
void voiceStart();
void voiceCapture();
void voiceCancel();
void voiceFinishSend();
void pollSerialReply();
void showWrapped(const String& title, const String& body);

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

  // NOTE: mic is NOT started here. i2s_driver_install wedges BTN1/GPIO0
  // while installed, so the mic is started lazily on Voice-page entry
  // (ensureMic) and stopped on exit (deinitMic). BTN1 works everywhere else.
  setupWifi(); // STA first, own AP if that fails
  setupWeb();

  beep(2);
  currentPage=0;
  showWelcomePage();
}

void loop() {
  static uint8_t prevBtn1 = HIGH, prevBtn2 = HIGH;
  static bool btn1Armed = false; // ignore BTN1 until seen HIGH once (kills reset-pulse phantom press)
  static unsigned long btn2DownAt = 0; // BTN2 press time: tap vs hold on voice page
  uint8_t btn1 = digitalRead(BTN1_PIN);
  uint8_t btn2 = digitalRead(BTN2_PIN);

  if (btn1 == HIGH) btn1Armed = true;

  // Pose integration runs every cycle regardless of page, so heading
  // doesn't get stale gaps when you're on the Eyes or Greeting page.
  updatePose();

  // Either button advances pages (BTN1/GPIO0 can wedge LOW when USB-serial
  // holds the auto-reset line or while the I2S mic runs, so BTN2 is a backup).
  // On the Voice page BTN2 is hold-to-talk / tap-to-exit (see below).
  bool advPress = false;
  if (btn1Armed && btn1 == LOW && prevBtn1 == HIGH) advPress = true;
  if (currentPage != VOICE_PAGE && btn2 == LOW && prevBtn2 == HIGH) advPress = true;

  if(advPress) {
    advancePage();
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
  } else if(currentPage == NEURAL_PAGE) {
    neuralDrive();
  } else if(currentPage == VOICE_PAGE) {
    // BTN2 hold-to-talk: record while held, send on release.
    // A quick tap (<400ms) exits to the next page instead — BTN1/GPIO0
    // reads dead while the I2S mic driver is installed, so BTN2 is the exit.
    if (btn2 == LOW && prevBtn2 == HIGH) btn2DownAt = millis();
    if (micOk && btn2 == LOW && !voiceRec && voiceStatus != V_WAITING) {
      voiceStart();
    }
    if (voiceRec) {
      voiceCapture();
      if (btn2 == HIGH) {
        if (millis() - btn2DownAt < 400) { voiceCancel(); advancePage(); } // tap → exit
        else voiceFinishSend(); // hold → send
      }
    } else if (btn2 == HIGH && prevBtn2 == LOW) {
      if (millis() - btn2DownAt < 400) advancePage(); // tap → exit
    }
    if (voiceStatus == V_WAITING && millis() - voiceWaitStart > 30000) {
      voiceStatus = V_NOLINK; // PC script not running
    }
    if (!voiceRec) showVoicePage();
  } else {
    stopMotors();
  }
  webServer.handleClient(); // control site, cheap to poll
  pollSerialReply(); // TXTHDR answers from the PC script, any page
}

// -------------------- Page navigation --------------------
void advancePage() {
  beep(1);
  if (voiceRec) voiceCancel(); // leaving the page mid-record drops it
  if (currentPage == VOICE_PAGE) deinitMic(); // free I2S: restores BTN1/GPIO0
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
    case 5:
      nnOutL = nnOutR = 0;
      nnPrevL = nnPrevR = 800;
      showNeuralPage();
      break;
    case 6: voiceStatus = V_IDLE; ensureMic(); showVoicePage(); break;
  }
  delay(250);
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

// Direct wheel speeds, negative = backwards. Used by the neural page where
// the two sides move independently instead of the usual forward/turn calls.
void setWheels(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, left >= 0 ? HIGH : LOW);
  digitalWrite(AIN2, left >= 0 ? LOW : HIGH);
  digitalWrite(BIN1, right >= 0 ? HIGH : LOW);
  digitalWrite(BIN2, right >= 0 ? LOW : HIGH);
  ledcWrite(pwmChannelA, abs(left));
  ledcWrite(pwmChannelB, abs(right));
  turnState = TURN_NONE;
  if (left > 0 && right > 0) driveState = DRIVE_FORWARD;
  else if (left < 0 && right < 0) driveState = DRIVE_BACKWARD;
  else if (left == 0 && right == 0) driveState = DRIVE_STOPPED;
  else driveState = DRIVE_TURNING;
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
    display.setCursor(4 + (i / 3) * 64, 38 + (i % 3) * 9);
    display.print(i+1);
    display.print(".");
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

void showNeuralPage() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("NEURAL DRIVE");
  display.printf("F %4.0fmm L %u R %u\n",
                 distanceFront * 10.0f, distLeft == 65535 ? 0 : distLeft,
                 distRight == 65535 ? 0 : distRight);
  display.printf("NN L %+.2f R %+.2f\n", nnOutL, nnOutR);
  display.printf("PWM %d %d\n", (int)(nnOutL * 200), (int)(nnOutR * 200));
  display.println(gyroYawRate > 5 ? "turning left" : gyroYawRate < -5 ? "turning right" : "straight");
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

// -------------------- Neural drive --------------------
// The net was trained offline (ml/train_policy.py) to copy a wall follower.
// It gets the three distances, the distance deltas, the gyro rate and its
// previous speed, and returns one wheel command per side.
static int netPwm(float v) {
  int p = (int)(v * 200.0f);
  if (p > 0 && p < 22) p = 22;
  if (p < 0 && p > -22) p = -22;
  return p;
}

void neuralDrive() {
  static unsigned long lastRead = 0, lastDraw = 0;
  if (millis() - lastRead < 40) return; // 25 Hz is plenty
  lastRead = millis();

  distLeft = sensorLeft.readRangeContinuousMillimeters();
  distRight = sensorRight.readRangeContinuousMillimeters();
  if (sensorLeft.timeoutOccurred()) distLeft = 65535;
  if (sensorRight.timeoutOccurred()) distRight = 65535;
  readUltrasonic();

  float f = (distanceFront > 0) ? distanceFront * 10.0f : 1200.0f;
  float l = (distLeft != 65535) ? (float)distLeft : 800.0f;
  float r = (distRight != 65535) ? (float)distRight : 800.0f;
  f = constrain(f, 0.0f, 1200.0f);
  l = constrain(l, 0.0f, 800.0f);
  r = constrain(r, 0.0f, 800.0f);

  float in[NN_IN] = {
    f / 1200.0f,
    l / 800.0f,
    r / 800.0f,
    constrain((r - l) / 800.0f, -1.0f, 1.0f),
    constrain((l - nnPrevL) / 150.0f, -1.0f, 1.0f),
    constrain((r - nnPrevR) / 150.0f, -1.0f, 1.0f),
    constrain(gyroYawRate / 120.0f, -1.0f, 1.0f),
    max(0.0f, (nnOutL + nnOutR) * 0.5f),
  };
  nnPrevL = l;
  nnPrevR = r;

  float out[NN_OUT];
  nnPolicy(in, out);

  nnOutL += (out[0] - nnOutL) * 0.5f;
  nnOutR += (out[1] - nnOutR) * 0.5f;

  if (distanceFront > 0 && distanceFront < 7.0f) {
    stopMotors(); // 70 mm, don't trust the net that close
  } else {
    setWheels(netPwm(nnOutL), netPwm(nnOutR));
  }

  if (millis() - lastDraw > 200) {
    lastDraw = millis();
    showNeuralPage();
  }
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
  // Throttle MPU reads to 50Hz: every-loop reads hammer a flaky I2C bus
  // (error flood) and 50Hz is plenty for gyro integration. dt is measured
  // between actual reads so no rotation is lost.
  if (nowMicros - lastPoseMicros < 20000) return;
  float dt = (nowMicros - lastPoseMicros) / 1000000.0f; // seconds
  lastPoseMicros = nowMicros;
  if (dt <= 0 || dt > 1.0f) return; // guard against overflow/first-run glitches

  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  float yawRateDegPerSec = (g.gyro.z * 180.0 / PI) - gyroZBiasDegPerSec;
  gyroYawRate = yawRateDegPerSec;
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

// -------------------- WiFi (STA first, own AP as fallback) --------------------
void setupWifi() {
  botIP = "no wifi";
  String ssid = WIFI_SSID;
  if (ssid.length() && ssid != "YOUR_WIFI_NAME") {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.printf("wifi: joining '%s'...\n", WIFI_SSID);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 12000) delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    botIP = WiFi.localIP().toString();
    Serial.printf("wifi ok: %s\n", botIP.c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    botIP = WiFi.softAPIP().toString();
    Serial.printf("AP up: '%s' ip %s\n", AP_SSID, botIP.c_str());
  }
}

// -------------------- Control website --------------------
String escHtml(const String& s) {
  String o = s;
  o.replace("&", "&amp;");
  o.replace("<", "&lt;");
  o.replace(">", "&gt;");
  return o;
}

void handleWebRoot() {
  String h = "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  h += "<style>body{font-family:sans-serif;text-align:center}td{padding:6px}a.btn{display:block;padding:14px 22px;background:#222;color:#fff;text-decoration:none;border-radius:8px;font-size:20px}</style></head><body>";
  h += "<h2>MazeRunner</h2>";
  h += "<table align='center'><tr><td></td><td><a class='btn' href='/cmd?dir=F'>F</a></td><td></td></tr>";
  h += "<tr><td><a class='btn' href='/cmd?dir=L'>L</a></td><td><a class='btn' href='/cmd?dir=S'>STOP</a></td><td><a class='btn' href='/cmd?dir=R'>R</a></td></tr>";
  h += "<tr><td></td><td><a class='btn' href='/cmd?dir=B'>B</a></td><td></td></tr></table>";
  h += "<form action='/speed'>Speed <input type='range' name='v' min='0' max='255' value='" + String(webSpeed) + "'>";
  h += "<input type='submit' value='Set'></form>";
  h += "<p>Sensors: <span id='s'>...</span></p>";
  h += "<p>Last voice: <b>" + escHtml(lastVoiceText) + "</b></p>";
  h += "<script>setInterval(async()=>{try{const r=await fetch('/sensors');const j=await r.json();document.getElementById('s').textContent=j.front+'cm / '+j.left+'mm / '+j.right+'mm';}catch(e){}},1000);</script>";
  h += "</body></html>";
  webServer.send(200, "text/html", h);
}

void webBurst(const String& dir) {
  if (dir == "F") { moveForward(webSpeed, webSpeed); delay(600); }
  else if (dir == "B") { moveBackward(webSpeed, webSpeed); delay(600); }
  else if (dir == "L") { turnLeftStart(webSpeed, webSpeed); delay(250); }
  else if (dir == "R") { turnRightStart(webSpeed, webSpeed); delay(250); }
  stopMotors();
}

void handleWebCmd() {
  webBurst(webServer.arg("dir"));
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleWebSpeed() {
  webSpeed = constrain(webServer.arg("v").toInt(), 0, 255);
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleWebSensors() {
  uint16_t l = sensorLeft.readRangeContinuousMillimeters();
  uint16_t r = sensorRight.readRangeContinuousMillimeters();
  if (sensorLeft.timeoutOccurred()) l = 65535;
  if (sensorRight.timeoutOccurred()) r = 65535;
  readUltrasonic();
  String j = "{\"front\":" + String(distanceFront, 1) + ",\"left\":" + String(l) +
             ",\"right\":" + String(r) + "}";
  webServer.send(200, "application/json", j);
}

void setupWeb() {
  webServer.on("/", handleWebRoot);
  webServer.on("/cmd", handleWebCmd);
  webServer.on("/speed", handleWebSpeed);
  webServer.on("/sensors", handleWebSensors);
  webServer.begin();
  Serial.println("web site up on port 80");
}

// -------------------- Mic (INMP441 over I2S) --------------------
// (micOk lives with the voice state at the top of the file.)

bool initMic() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // L/R tied to GND
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  if (i2s_driver_install(I2S_PORT, &cfg, 0, NULL) != ESP_OK) return false;
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) return false;
  i2s_zero_dma_buffer(I2S_PORT);
  micOk = true;
  Serial.println("mic ready: SCK=10 WS=11 SD=12 @16kHz");
  return true;
}

// Starts the mic on Voice-page entry. The I2S driver wedges BTN1/GPIO0
// while installed, so it only runs while the voice page is showing.
void ensureMic() {
  if (micOk) return;
  if (initMic()) Serial.println("mic on (voice page)");
  else Serial.println("mic lazy init failed");
}

// Stops the mic when leaving the Voice page — frees I2S, restores BTN1.
void deinitMic() {
  if (voiceRec) voiceCancel();
  if (micOk) {
    i2s_driver_uninstall(I2S_PORT);
    Serial.println("mic off, BTN1 back");
  }
  micOk = false;
}

// -------------------- Voice page --------------------
void showWrapped(const String& title, const String& body) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(title);
  int x = 0, y = 10;
  String word = "";
  auto flushWord = [&]() {
    if (!word.length()) return;
    int w = word.length() * 6;
    if (x + w > SCREEN_WIDTH) { x = 0; y += 9; }
    if (y > SCREEN_HEIGHT - 9) return;
    display.setCursor(x, y);
    display.print(word);
    x += w + 6;
    word = "";
  };
  for (unsigned i = 0; i < body.length(); i++) {
    char c = body[i];
    if (c == ' ' || c == '\n') flushWord();
    else word += c;
    if (c == '\n') { x = 0; y += 9; }
    if (y > SCREEN_HEIGHT - 9) break;
  }
  flushWord();
  display.display();
}

void showVoicePage() {
  if (!micOk) {
    showWrapped("VOICE", "mic init failed. check wiring: SCK=10 WS=11 SD=12");
    return;
  }
  if (voiceStatus == V_RECORDING) return; // progress drawn by voiceCapture
  if (voiceStatus == V_WAITING) {
    showWrapped("VOICE", "thinking... (PC transcribes + asks ollama)");
    return;
  }
  if (voiceStatus == V_NOLINK) {
    showWrapped("VOICE", "no reply. start mazerunner_voice.py on the PC, then hold BTN2 again.");
    return;
  }
  if (voiceStatus == V_RESULT) {
    String both = voiceHeard;
    if (voiceReply.length()) both += String("\nAI: ") + voiceReply;
    showWrapped("YOU SAID", both);
    return;
  }
  // idle
  String msg = "Hold BTN2, speak, release.\nIP: " + botIP;
  if (lastVoiceText.length()) msg += String("\nLast: ") + lastVoiceText;
  showWrapped("VOICE", msg);
}

void voiceStart() {
  if (!micOk) return;
  size_t maxSamples = voiceMaxSec() * I2S_SAMPLE_RATE;
  if (voiceBuf) { free(voiceBuf); voiceBuf = nullptr; }
  voiceBuf = (int16_t *)malloc(maxSamples * sizeof(int16_t));
  if (!voiceBuf) {
    voiceStatus = V_IDLE;
    showWrapped("VOICE", "out of memory for recording");
    return;
  }
  voiceCap = maxSamples;
  voiceLen = 0;
  voiceRec = true;
  voiceRecStart = millis();
  voiceStatus = V_RECORDING;
  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("voice: recording, release BTN2 to send");
}

void voiceCapture() {
  if (!voiceRec || !voiceBuf) return;
  int32_t tmp[I2S_READ_SAMPLES];
  size_t got = 0;
  if (i2s_read(I2S_PORT, tmp, sizeof(tmp), &got, pdMS_TO_TICKS(20)) == ESP_OK) {
    size_t n = got / sizeof(int32_t);
    for (size_t i = 0; i < n && voiceLen < voiceCap; i++) {
      int32_t s = tmp[i] >> 11;
      if (s > 32767) s = 32767;
      if (s < -32768) s = -32768;
      voiceBuf[voiceLen++] = (int16_t)s;
    }
  }
  if (voiceLen >= voiceCap) { voiceFinishSend(); return; } // hit the cap
  static unsigned long lastProg = 0;
  if (millis() - lastProg > 300) {
    lastProg = millis();
    int secs = (millis() - voiceRecStart) / 1000;
    int pct = voiceCap ? (int)(100 * voiceLen / voiceCap) : 0;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("LISTENING...");
    display.drawRect(0, 20, 110, 10, SSD1306_WHITE);
    display.fillRect(1, 21, constrain(pct * 108 / 100, 0, 108), 8, SSD1306_WHITE);
    display.setCursor(0, 40);
    display.printf("%ds - release BTN2", secs);
    display.display();
  }
}

void voiceCancel() {
  voiceRec = false;
  if (voiceBuf) { free(voiceBuf); voiceBuf = nullptr; }
  voiceCap = voiceLen = 0;
  voiceStatus = V_IDLE;
}

void voiceFinishSend() {
  voiceRec = false;
  uint32_t n = (uint32_t)(voiceLen * sizeof(int16_t));
  Serial.print("AUDHDR");
  Serial.write((const uint8_t *)&n, 4);
  Serial.flush();
  const uint8_t *p = (const uint8_t *)voiceBuf;
  size_t left = n;
  while (left) {
    size_t chunk = left > 512 ? 512 : left;
    size_t w = Serial.write(p, chunk);
    if (w == 0) break;
    p += w; left -= w;
  }
  Serial.println();
  Serial.printf("AUDEND %u\n", (unsigned)n);
  voiceStatus = V_WAITING;
  voiceWaitStart = millis();
  showVoicePage();
}

// PC answers "TXTHDR" + uint32le len + utf8 ("YOU: ...\nAI: ..."). Runs every loop.
void pollSerialReply() {
  static uint8_t stage = 0; // 0 = magic, 1 = len, 2 = payload
  static uint8_t mi = 0, li = 0;
  static uint32_t tlen = 0;
  static String payload = "";
  const char *magic = "TXTHDR";
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (stage == 0) {
      if (c == magic[mi]) { mi++; if (mi == 6) { stage = 1; li = 0; tlen = 0; } }
      else mi = (c == magic[0]) ? 1 : 0;
    } else if (stage == 1) {
      tlen |= ((uint32_t)c << (8 * li)); li++;
      if (li == 4) {
        if (tlen > 512) tlen = 512;
        payload = "";
        if (tlen == 0) { stage = 0; mi = 0; }
        else stage = 2;
      }
    } else {
      payload += (char)c;
      if (payload.length() >= tlen) {
        int k = payload.indexOf("\nAI:");
        if (k >= 0) {
          voiceHeard = payload.substring(0, k);
          voiceReply = payload.substring(k + 4);
        } else {
          voiceHeard = payload;
          voiceReply = "";
        }
        voiceHeard.trim(); voiceReply.trim();
        voiceHeard.replace("YOU:", "");
        voiceHeard.trim();
        lastVoiceText = voiceHeard + (voiceReply == "" ? "" : " -> " + voiceReply);
        voiceStatus = V_RESULT;
        if (currentPage == VOICE_PAGE) showVoicePage();
        stage = 0; mi = 0;
      }
    }
  }
}
