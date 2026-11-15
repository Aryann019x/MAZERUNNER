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
#include <HTTPClient.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
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
#ifndef OTA_PASS
#define OTA_PASS "mazerunner"
#endif

// IP of the laptop running server.py. Can be changed from the web page.
#ifndef LAPTOP_HOST
#define LAPTOP_HOST ""
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

// Both motor leads are wired opposite to the TB6612's usual "forward"
// combination: IN1=HIGH, IN2=LOW drives the wheels backwards on this robot.
// These are the levels that actually move the robot toward the HC-SR04 end.
// If the motor plugs are ever swapped, flip FWD1/FWD2 and REV1/REV2.
const int MOTOR_FWD1 = LOW;
const int MOTOR_FWD2 = HIGH;
const int MOTOR_REV1 = HIGH;
const int MOTOR_REV2 = LOW;

// Translation-only direction trim, saved in NVS. The spin directions are
// calibrated separately, so this must not touch setSpinPins/setWheels.
bool driveInvert = false;

inline void setTranslateDir(bool forward) {
  bool f = forward != driveInvert;
  digitalWrite(AIN1, f ? MOTOR_FWD1 : MOTOR_REV1);
  digitalWrite(AIN2, f ? MOTOR_FWD2 : MOTOR_REV2);
  digitalWrite(BIN1, f ? MOTOR_FWD1 : MOTOR_REV1);
  digitalWrite(BIN2, f ? MOTOR_FWD2 : MOTOR_REV2);
}

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
// 4 = Pose Telemetry, 5 = Neural Drive, 6 = Laptop, 7 = Voice
const uint8_t totalPages = 8;
const char* pageNames[totalPages - 1] = {"Greet", "Eyes", "Drive", "Pose", "Neural", "Laptop", "Voice"};
const uint8_t NEURAL_PAGE = 5;
const uint8_t LAPTOP_PAGE = 6;
const uint8_t VOICE_PAGE = 7;
uint8_t currentPage = 0;

VL53L0X sensorLeft;
VL53L0X sensorRight;
float distanceFront = -1;
uint16_t distLeft = 65535;
uint16_t distRight = 65535;
unsigned long frontEchoUs = 0;
bool frontEchoValid = false;
float lastAx = 0, lastAy = 0, lastAz = 0;
float lastGx = 0, lastGy = 0, lastGz = 0;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_NeoPixel pixels(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
WebServer webServer(80);

// -------------------- Web control state --------------------
int webSpeed = 130; // 0..255, manual drive speed
char webHoldDir = 'S'; // F/B/L/R while a page button is held, S = stopped
unsigned long webHoldLast = 0;
bool otaReady = false;
bool netServicesStarted = false;
unsigned long wifiRetryAt = 0;
String apIP = "192.168.4.1";
String wifiSsid = WIFI_SSID;
String wifiPass = WIFI_PASS;
String lastVoiceText = ""; // last YOU/AI text from the PC script, shown on site + voice page
String botIP = "no wifi";
String laptopHost = LAPTOP_HOST; // server.py machine, settable from the web page
String lastAIAnswer = "";
String lastLaptopAction = "idle";
String lastLaptopThought = "";
Preferences prefs;

// -------------------- Voice page state --------------------
static bool micOk = false; // I2S mic installed? (lazy: only while on voice page)
static uint8_t *voiceBufAlloc = nullptr; // 44 byte wav header + pcm
static int16_t *voiceBuf = nullptr;      // points past the header
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
// Spins are closed-loop on the gyro so a 90 degree turn is actually 90
// degrees instead of "however far the wheels got in 190 ms".
bool turnActive = false;
bool turnCcw = true;
bool turnPolarityChecked = false;
bool turnFault = false; // gyro says the spin went the wrong way -> stop, don't flip
uint8_t turnSpeedPwm = 0;
float turnStartHeading = 0;
float turnTargetDeg = 90;
unsigned long turnStartTime = 0;
const unsigned long TURN_MAX_MS = 3200; // stuck against something -> give up

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

// Left-wheel scale learned from the gyro. 1.0 = no correction. The motors
// are never perfectly matched, so straight commands slowly drift; this trims
// the faster side until the yaw rate sits near zero. Saved in NVS.
float motorBalance = 1.0f;
bool balanceDirty = false;
unsigned long lastBalanceSave = 0;
bool straightCmd = false;
float trimYawSum = 0, trimRunTime = 0;
int pwmLeft = 0, pwmRight = 0; // last commanded wheel PWM, used for rough odometry

// Bench straight-line test (!S): drives at a fixed PWM for 2 s without any
// steering, so the heading change shows the raw wheel imbalance.
bool straightTestActive = false;
unsigned long straightTestStart = 0;
float straightTestHeading = 0;
int straightTestPwm = 100;

// One-wheel test (!1..!4): isolates motor vs driver vs traction.
int wheelTestId = 0;
unsigned long wheelTestStart = 0;
float wheelTestHeading = 0;
const int WHEEL_TEST_PWM = 120;

// TUNE THESE: run moveForward(forwardSpeed,forwardSpeed) for exactly 2s on a
// known-length straight run, measure distance traveled in mm, divide by 2.
// Repeat for backward. These placeholders are almost certainly wrong for your robot.
const float FORWARD_SPEED_MM_S = 250.0;
const float BACKWARD_SPEED_MM_S = 250.0;
const float SPEED_CAL_PWM = 150.0; // speed constants were measured at this PWM

enum DriveState { DRIVE_STOPPED, DRIVE_FORWARD, DRIVE_BACKWARD, DRIVE_TURNING };
DriveState driveState = DRIVE_STOPPED;

const char* driveStateName() {
  switch (driveState) {
    case DRIVE_STOPPED: return "stop";
    case DRIVE_FORWARD: return "fwd";
    case DRIVE_BACKWARD: return "back";
    case DRIVE_TURNING: return "turn";
  }
  return "?";
}

// Neural page state
static float nnOutL = 0, nnOutR = 0;
static float nnPrevL = 800, nnPrevR = 800;

// Debounce the front readings: a single echo dropout should not start a turn.
int frontObstacleStreak = 0;

// Function declarations
void beep(uint8_t times=1);
void startMelody();
void updateMelody();
void moveForward(uint8_t left, uint8_t right);
void moveBackward(uint8_t left, uint8_t right);
void startTurn(bool ccw, uint8_t speed, float degrees);
void spinCCW(uint8_t speed, unsigned long ms);
void spinCW(uint8_t speed, unsigned long ms);
void brakeMotors(uint16_t ms);
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
void showLaptopPage();
void laptopDrive();
void loadLaptopSettings();
String httpPostJson(const String& url, const String& body, int* code);
String httpPostWav(const String& url, const uint8_t* data, size_t len, int* code);
String jsonField(const String& js, const char* key);
String escJson(const String& s);
void ultrasonicHybridDrive();
void autoReset();
void startStraightTest(int pwm, bool forward = true);
void straightTestUpdate();
void startWheelTest(int id);
void wheelTestUpdate();
void saveDriveInvert();
void sensorPowerDown(int pin);
void sensorPowerUp(int pin);
void initSensors();
void readUltrasonic();
void updateOLEDVL53(float front, uint16_t left, uint16_t right, const char* motorState);
void calibrateGyroBias();
void updatePose();
void predictPosition(float secondsAhead, float &predX, float &predY);
void loadMotorBalance();
void saveMotorBalance();
int balancedLeft(int pwm);
void calibrateMotorBalance();
void handleWebTrim();
void showVoicePage();
void setupWifi();
void setupWeb();
void handleWebRoot();
void handleWebCmd();
void handleWebSpeed();
void handleWebSensors();
void handleWebAsk();
void handleWebTarget();
void webBurst(const String& dir);
bool initMic();
void ensureMic();
void deinitMic();
void advancePage();
void voiceStart();
void voiceCapture();
void voiceCancel();
void voiceFinishSend();
void voiceSendWifi();
void pollSerialReply();
void handleSerialCmd(char c);
void showWrapped(const String& title, const String& body);
void webDriveApply();
void wifiLoop();

void setup() {
  Serial.begin(115200);

  // Both ToF sensors come up at the same address and can hold the shared
  // I2C bus low. Park them in shutdown before the OLED or MPU talk, or the
  // whole bus looks dead and setup() stops at "MPU6050 not found".
  sensorPowerDown(XSHUT_PIN1);
  sensorPowerDown(XSHUT_PIN2);
  delay(20);

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
  loadLaptopSettings();
  loadMotorBalance();
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

  if (webHoldDir != 'S') {
    webDriveApply();
  } else if (wheelTestId) {
    wheelTestUpdate();
  } else if (straightTestActive) {
    straightTestUpdate();
  } else if(currentPage == 1) {
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
  } else if(currentPage == LAPTOP_PAGE) {
    laptopDrive();
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
  wifiLoop(); // background hotspot join / retry
  if (otaReady) ArduinoOTA.handle();
  pollSerialReply(); // TXTHDR answers from the PC script, any page
}

// -------------------- Page navigation --------------------
void advancePage() {
  beep(1);
  stopMotors(); // don't leave the last PWM running on a non-driving page
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
    case 3: autoReset(); break; // Autonomous Drive renders itself continuously below
    case 4: showPosePage(); break;
    case 5:
      nnOutL = nnOutR = 0;
      nnPrevL = nnPrevR = 800;
      showNeuralPage();
      break;
    case 6:
      stopMotors();
      showLaptopPage();
      break;
    case 7: voiceStatus = V_IDLE; ensureMic(); showVoicePage(); break;
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
        left_eye_x = constrain(left_eye_x + random(-2, 3), baseLeftEyeX - eyeOffsetMax, baseLeftEyeX + eyeOffsetMax);
        left_eye_y = constrain(left_eye_y + random(-2, 3), baseLeftEyeY - eyeOffsetMax, baseLeftEyeY + eyeOffsetMax);
        right_eye_x = constrain(right_eye_x + random(-2, 3), baseRightEyeX - eyeOffsetMax, baseRightEyeX + eyeOffsetMax);
        right_eye_y = constrain(right_eye_y + random(-2, 3), baseRightEyeY - eyeOffsetMax, baseRightEyeY + eyeOffsetMax);
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
  int targetX = constrain(map((int)pitch, -30, 30, -eyeOffsetMax, eyeOffsetMax), -eyeOffsetMax, eyeOffsetMax);
  int targetY = constrain(map((int)roll, -30, 30, -eyeOffsetMax, eyeOffsetMax), -eyeOffsetMax, eyeOffsetMax);
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
// Motor A is the physical LEFT wheel, motor B is the RIGHT one. The motor
// leads are also wired opposite to the TB6612 convention, so every direction
// write goes through the MOTOR_FWD* and MOTOR_REV* levels defined above. The
// learned trim scales the left (A) side; if the sides are ever swapped, the
// robot will curve away instead of straightening out.
int balancedLeft(int pwm) {
  return constrain((int)lroundf(pwm * motorBalance), 0, 255);
}

void moveForward(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  setTranslateDir(true);
  ledcWrite(pwmChannelA, balancedLeft(left));
  ledcWrite(pwmChannelB, right);
  pwmLeft = balancedLeft(left);
  pwmRight = right;
  straightCmd = (left == right && left > 0);
  driveState = (left == 0 && right == 0) ? DRIVE_STOPPED : DRIVE_FORWARD;
}

void moveBackward(uint8_t left, uint8_t right) {
  digitalWrite(STBY, HIGH);
  setTranslateDir(false);
  ledcWrite(pwmChannelA, balancedLeft(left));
  ledcWrite(pwmChannelB, right);
  pwmLeft = balancedLeft(left);
  pwmRight = right;
  straightCmd = (left == right && left > 0);
  driveState = (left == 0 && right == 0) ? DRIVE_STOPPED : DRIVE_BACKWARD;
}

void setSpinPins(bool ccw, uint8_t speed) {
  digitalWrite(STBY, HIGH);
  if (ccw) {
    // Nose left: left wheel (A) back, right wheel (B) forward.
    digitalWrite(AIN1, MOTOR_REV1);
    digitalWrite(AIN2, MOTOR_REV2);
    digitalWrite(BIN1, MOTOR_FWD1);
    digitalWrite(BIN2, MOTOR_FWD2);
  } else {
    // Nose right: left wheel (A) forward, right wheel (B) back.
    digitalWrite(AIN1, MOTOR_FWD1);
    digitalWrite(AIN2, MOTOR_FWD2);
    digitalWrite(BIN1, MOTOR_REV1);
    digitalWrite(BIN2, MOTOR_REV2);
  }
  ledcWrite(pwmChannelA, speed);
  ledcWrite(pwmChannelB, speed);
  pwmLeft = pwmRight = speed;
}

// Starts a gyro-closed-loop spin. ccw = nose left (positive gyro yaw).
// A is the left wheel and B is the right one. CCW means the right wheel goes
// forward and the left goes backward; CW is the reverse. The MOTOR_FWD*/
// MOTOR_REV* levels already account for this robot's reversed motor leads.
// If the gyro ever sees the chassis rotating the wrong way, turnUpdate stops
// the spin and raises turnFault instead of flipping directions mid-turn.
void startTurn(bool ccw, uint8_t speed, float degrees) {
  setSpinPins(ccw, speed);
  straightCmd = false;
  turnCcw = ccw;
  turnTargetDeg = degrees;
  turnStartHeading = pose.heading;
  turnStartTime = millis();
  turnActive = true;
  turnPolarityChecked = false;
  turnFault = false;
  turnSpeedPwm = speed;
  driveState = DRIVE_TURNING;
}

// Short brake: both inputs high on the TB6612. Coasting after a spin is what
// made the old timed turns overshoot depending on the floor.
void brakeMotors(uint16_t ms) {
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, HIGH);
  ledcWrite(pwmChannelA, 255);
  ledcWrite(pwmChannelB, 255);
  delay(ms);
  stopMotors();
}

// Blocking manual spins for the web/laptop buttons. Auto mode uses startTurn
// plus turnUpdate so the loop keeps running.
void spinCCW(uint8_t speed, unsigned long ms) {
  startTurn(true, speed, 999);
  delay(ms);
  brakeMotors(30);
}

void spinCW(uint8_t speed, unsigned long ms) {
  startTurn(false, speed, 999);
  delay(ms);
  brakeMotors(30);
}

// Non-blocking: returns true while the spin is still in progress.
bool turnUpdate() {
  if (!turnActive) return false;
  float delta = pose.heading - turnStartHeading;
  while (delta > 180) delta -= 360;
  while (delta < -180) delta += 360;
  float progress = turnCcw ? delta : -delta;
  if (!turnPolarityChecked && millis() - turnStartTime > 300) {
    turnPolarityChecked = true;
    if (progress < -1.5f) {
      // A wrong-way spin in a 15 cm corridor can put a corner through the
      // wall. Stop and let auto halt instead of flipping the motor pins.
      Serial.println("turn polarity fault");
      brakeMotors(40);
      turnActive = false;
      turnFault = true;
      return false;
    }
  }
  if (progress >= turnTargetDeg || millis() - turnStartTime >= TURN_MAX_MS) {
    bool stuck = (millis() - turnStartTime >= TURN_MAX_MS);
    brakeMotors(45);
    turnActive = false;
    if (stuck) Serial.println("turn timeout");
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
  straightCmd = false;
  pwmLeft = pwmRight = 0;
  driveState = DRIVE_STOPPED;
  turnActive = false;
}

// Direct wheel speeds, negative = backwards. Used by the neural page where
// the two sides move independently instead of the usual forward/turn calls.
// A is the left motor and B is the right one.
void setWheels(int left, int right) {
  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);
  int l = (left >= 0) ? balancedLeft(left) : -balancedLeft(-left);
  digitalWrite(STBY, HIGH);
  digitalWrite(AIN1, l >= 0 ? MOTOR_FWD1 : MOTOR_REV1);
  digitalWrite(AIN2, l >= 0 ? MOTOR_FWD2 : MOTOR_REV2);
  digitalWrite(BIN1, right >= 0 ? MOTOR_FWD1 : MOTOR_REV1);
  digitalWrite(BIN2, right >= 0 ? MOTOR_FWD2 : MOTOR_REV2);
  ledcWrite(pwmChannelA, abs(l));
  ledcWrite(pwmChannelB, abs(right));
  pwmLeft = abs(l);
  pwmRight = abs(right);
  straightCmd = (left == right && left != 0);
  turnActive = false;
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
  display.setCursor(0, 24);
  display.print(botIP == "192.168.4.1" ? "AP " : "WiFi ");
  display.println(botIP);
  for(uint8_t i=0; i<totalPages-1; i++) {
    display.setCursor(2 + (i / 4) * 64, 32 + (i % 4) * 8);
    display.print(i+1);
    display.print(".");
    display.print(pageNames[i]);
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
  display.printf("Trim L: %+.1f%%\n", (motorBalance - 1.0f) * 100.0f);
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
  frontEchoUs = duration;
  frontEchoValid = (duration > 0);
  distanceFront = frontEchoValid ? (duration / 2.0) / 29.1 : -1;
}

// -------------------- Autonomous maze drive --------------------
// The cardboard maze uses 15 cm cells with a robot that is almost as wide as
// the corridor, plus a couple of 12 cm dead ends. There is no room to spin
// inside a cell, so this mode drives straight with a gyro heading hold and
// only spins in a side room (the 24/41/55 cm spaces) where the chassis can
// sweep. When the front is blocked and both sides are corridor-tight, it
// reverses to the last room instead of pivoting into a wall.
enum AutoState { AUTO_WAIT, AUTO_CRUISE, AUTO_TURN, AUTO_BACK, AUTO_HALT };

AutoState autoState = AUTO_WAIT;
unsigned long autoStateStart = 0;
unsigned long autoCruiseStart = 0; // when the current straight run began
unsigned long autoRoomSeenAt = 0;
float autoHeading = 0;         // heading to hold while cruising
bool autoTurnCcw = true;
bool autoBackSawRoom = false;
bool autoSawFront = false;
unsigned long autoNoFrontSince = 0;
unsigned long autoStuckSince = 0;
unsigned long autoTurnWindowStart = 0;
int autoTurnCount = 0;

const int AUTO_SPEED = 125;        // forward PWM, above the motors' dead zone
const int AUTO_BACK_SPEED = 110;
const int AUTO_TURN_SPEED = 90;
const float AUTO_FRONT_STOP_CM = 9.0f;
const float AUTO_FRONT_PANIC_CM = 5.5f;
const float AUTO_TURN_CLEAR_CM = 4.0f; // below this after a spin = real collision
const int AUTO_SIDE_ROOM_MM = 55;  // clearance that means a real side room
const unsigned long AUTO_BACK_MAX_MS = 3200;
const unsigned long AUTO_WAIT_MS = 900;
const unsigned long AUTO_BACK_ROOM_SETTLE_MS = 900;
const unsigned long AUTO_SOFT_START_MS = 300;

// Flip if a hand test shows the left reading move when the right wall is
// touched. The maze code is only safe when this matches the wiring.
const int AUTO_SIDE_SIGN = 1;

float wrap180(float a) {
  while (a > 180) a -= 360;
  while (a < -180) a += 360;
  return a;
}

const char* autoStateName() {
  switch (autoState) {
    case AUTO_WAIT: return "wait";
    case AUTO_CRUISE: return "fwd";
    case AUTO_TURN: return autoTurnCcw ? "turn L" : "turn R";
    case AUTO_BACK: return "back";
    case AUTO_HALT: return "halt";
  }
  return "?";
}

void autoReset() {
  stopMotors();
  autoState = AUTO_WAIT;
  autoStateStart = millis();
  autoRoomSeenAt = 0;
  autoBackSawRoom = false;
  autoSawFront = false;
  autoHeading = pose.heading;
  autoCruiseStart = millis();
  frontObstacleStreak = 0;
  autoNoFrontSince = 0;
  autoStuckSince = 0;
  autoTurnCount = 0;
  autoTurnWindowStart = millis();
}

void startStraightTest(int pwm, bool forward) {
  stopMotors();
  straightTestPwm = constrain(pwm, 60, 160);
  currentPage = 0;
  straightTestActive = true;
  straightTestStart = millis();
  straightTestHeading = pose.heading;
  if (forward) moveForward(straightTestPwm, straightTestPwm);
  else moveBackward(straightTestPwm, straightTestPwm);
  straightCmd = false; // keep the trim learner out of the measurement
  Serial.printf("STRAIGHT start pwm=%d %s H=%.1f\n", straightTestPwm,
                forward ? "fwd" : "back", straightTestHeading);
}

void straightTestUpdate() {
  if (!straightTestActive) return;
  if (millis() - straightTestStart < 2000) return;
  stopMotors();
  straightTestActive = false;
  readUltrasonic();
  Serial.printf("STRAIGHT end dh=%.1f F=%.1f BAL=%.3f\n",
                wrap180(pose.heading - straightTestHeading), distanceFront, motorBalance);
}

void startWheelTest(int id) {
  stopMotors();
  currentPage = 0;
  wheelTestId = id;
  wheelTestStart = millis();
  wheelTestHeading = pose.heading;
  const char *label = "?";
  switch (id) {
    case 1: setWheels(-WHEEL_TEST_PWM, 0); label = "left fwd"; break;
    case 2: setWheels(WHEEL_TEST_PWM, 0); label = "left rev"; break;
    case 3: setWheels(0, -WHEEL_TEST_PWM); label = "right fwd"; break;
    case 4: setWheels(0, WHEEL_TEST_PWM); label = "right rev"; break;
    default: wheelTestId = 0; return;
  }
  showWrapped("WHEEL TEST", String(id) + " " + label);
  Serial.printf("WHEELTEST start %d %s\n", id, label);
}

void wheelTestUpdate() {
  if (!wheelTestId) return;
  if (millis() - wheelTestStart < 900) return;
  stopMotors();
  readUltrasonic();
  Serial.printf("WHEELTEST end %d dh=%.1f F=%.1f L=%u R=%u\n", wheelTestId,
                wrap180(pose.heading - wheelTestHeading), distanceFront, distLeft, distRight);
  wheelTestId = 0;
  showWelcomePage();
}

void startAutoTurn(bool ccw) {
  unsigned long now = millis();
  if (now - autoTurnWindowStart > 7000) {
    autoTurnWindowStart = now;
    autoTurnCount = 0;
  }
  autoTurnCount++;
  if (autoTurnCount > 3) {
    stopMotors();
    autoState = AUTO_HALT;
    beep(3);
    Serial.println("AUTO halt: turn loop");
    return;
  }
  stopMotors();
  autoTurnCcw = ccw;
  startTurn(ccw, AUTO_TURN_SPEED, 88.0f); // brake/coast finishes the last couple of degrees
  autoState = AUTO_TURN;
  autoStateStart = millis();
}

void showAutoStatus() {
  static unsigned long lastDraw = 0, lastLog = 0;
  unsigned long now = millis();
  if (now - lastDraw > 150) {
    lastDraw = now;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.printf("AUTO %s\n", autoStateName());
    display.print("F ");
    if (distanceFront > 0) display.printf("%.0fcm\n", distanceFront);
    else display.println("out");
    display.print("L ");
    if (distLeft == 65535) display.print("out"); else display.printf("%dmm", distLeft);
    display.print("  R ");
    if (distRight == 65535) display.print("out"); else display.printf("%dmm", distRight);
    display.println();
    display.printf("Hdg %+.0f", pose.heading);
    display.display();
  }
  if (now - lastLog > 300) {
    lastLog = now;
    Serial.printf("AUTO %s F=%.1f L=%u R=%u H=%.1f st=%d\n",
                  autoStateName(), distanceFront, distLeft, distRight,
                  pose.heading, (int)autoState);
  }
}

void ultrasonicHybridDrive() {
  // A spin is gyro-owned: don't make new decisions until it finishes.
  if (autoState == AUTO_TURN) {
    if (turnUpdate()) {
      showAutoStatus();
      return;
    }
    if (turnFault) {
      stopMotors();
      autoState = AUTO_HALT;
      beep(2);
      Serial.println("AUTO halt: turn polarity fault");
      showAutoStatus();
      return;
    }
    // Re-read the front before trusting the new direction. A corridor wall
    // 7.5 cm from a freshly turned nose is normal, so only treat it as a
    // collision when it is closer than the chassis can recover from.
    readUltrasonic();
    if (distanceFront > 0 && distanceFront < AUTO_TURN_CLEAR_CM) {
      stopMotors();
      autoState = AUTO_BACK;
      autoStateStart = millis();
      autoBackSawRoom = false;
      autoRoomSeenAt = 0;
      Serial.println("AUTO turn hit wall");
    } else {
      autoHeading = wrap180(autoHeading + (autoTurnCcw ? 90.0f : -90.0f));
      autoCruiseStart = millis();
      autoState = AUTO_CRUISE;
    }
    showAutoStatus();
    return;
  }

  distLeft = sensorLeft.readRangeContinuousMillimeters();
  distRight = sensorRight.readRangeContinuousMillimeters();
  if (sensorLeft.timeoutOccurred()) distLeft = 65535;
  if (sensorRight.timeoutOccurred()) distRight = 65535;
  readUltrasonic();

  bool frontKnown = (distanceFront > 0);
  bool frontClose = (frontKnown && distanceFront < AUTO_FRONT_STOP_CM);
  bool leftRoom  = (distLeft  != 65535 && distLeft  > AUTO_SIDE_ROOM_MM);
  bool rightRoom = (distRight != 65535 && distRight > AUTO_SIDE_ROOM_MM);

  // A dead or unplugged HC-SR04 would otherwise look like "wide open" and the
  // robot would drive into the first wall it met. The maze is never 1.7 m
  // clear in a straight line, so a long run of no echo is a fault.
  if (autoState == AUTO_CRUISE || autoState == AUTO_BACK) {
    if (!frontKnown) {
      if (autoNoFrontSince == 0) autoNoFrontSince = millis();
      if (millis() - autoNoFrontSince > 700) {
        stopMotors();
        autoState = AUTO_HALT;
        beep(2);
        Serial.println("AUTO halt: front sensor dead");
        showAutoStatus();
        return;
      }
    } else {
      autoNoFrontSince = 0;
    }
  } else {
    autoNoFrontSince = 0;
  }

  switch (autoState) {
    case AUTO_WAIT:
      stopMotors();
      if (frontKnown) autoSawFront = true;
      if (millis() - autoStateStart >= AUTO_WAIT_MS) {
        if (!autoSawFront) {
          autoState = AUTO_HALT;
          beep(2);
          Serial.println("AUTO halt: front sensor dead at start");
          break;
        }
        autoHeading = pose.heading;
        autoCruiseStart = millis();
        autoState = AUTO_CRUISE;
      }
      break;

    case AUTO_CRUISE: {
      if (frontClose) {
        frontObstacleStreak++;
      } else {
        frontObstacleStreak = 0;
      }

      if (frontObstacleStreak >= 2 || (frontKnown && distanceFront < AUTO_FRONT_PANIC_CM)) {
        stopMotors();
        // The nose is already close to the wall, so a 90 degree sweep from
        // here drags a front corner through it even when a side room is
        // visible. Always back first; AUTO_BACK picks the room side and spins
        // only after the chassis centre is clear of the corridor mouth.
        autoState = AUTO_BACK;
        autoStateStart = millis();
        autoBackSawRoom = false;
        autoRoomSeenAt = 0;
        break;
      }

      // Gyro heading hold + gentle side centring. Both are small so a noisy
      // ToF sample cannot yank the wheel hard in a 15 cm corridor.
      float err = wrap180(autoHeading - pose.heading);
      int steer = constrain((int)(err * 1.4f), -22, 22);
      if (distLeft != 65535 && distRight != 65535 &&
          distLeft >= 15 && distRight >= 15 &&
          distLeft <= 250 && distRight <= 250 &&
          abs((int)distLeft - (int)distRight) < 80) {
        int centring = (int)(((int)distLeft - (int)distRight) * 0.12f) * AUTO_SIDE_SIGN;
        steer += constrain(centring, -10, 10);
      }
      steer = constrain(steer, -26, 26);
      // If the nose is badly off-heading and the gyro sees no rotation, the
      // chassis is wedged on a wall. Stop instead of grinding along it.
      if (fabsf(err) > 35.0f && fabsf(gyroYawRate) < 2.0f) {
        if (autoStuckSince == 0) autoStuckSince = millis();
        if (millis() - autoStuckSince > 1200) {
          stopMotors();
          autoState = AUTO_HALT;
          beep(3);
          Serial.println("AUTO halt: wedged");
          break;
        }
      } else {
        autoStuckSince = 0;
      }
      // Short soft start: the first PWM step is what breaks traction and
      // starts the old sideways lurch. Ramp 90 -> 125 over 300 ms.
      int base = AUTO_SPEED;
      unsigned long runFor = millis() - autoCruiseStart;
      if (runFor < AUTO_SOFT_START_MS) {
        base = 90 + (AUTO_SPEED - 90) * (int)runFor / (int)AUTO_SOFT_START_MS;
      }
      int l = constrain(base - steer, 50, 150);
      int r = constrain(base + steer, 50, 150);
      moveForward(l, r);
      break;
    }

    case AUTO_BACK: {
      moveBackward(AUTO_BACK_SPEED, AUTO_BACK_SPEED);
      unsigned long elapsed = millis() - autoStateStart;

      if (!autoBackSawRoom && elapsed > 150 && (leftRoom || rightRoom)) {
        autoBackSawRoom = true;
        autoRoomSeenAt = millis();
      }
      // Keep backing a little after the rear pod sees the room so the centre
      // of the chassis clears the corridor before the spin.
      if (autoBackSawRoom && millis() - autoRoomSeenAt >= AUTO_BACK_ROOM_SETTLE_MS) {
        stopMotors();
        bool goLeft = leftRoom && (!rightRoom || distLeft >= distRight);
        if (goLeft || rightRoom) startAutoTurn(goLeft);
        else { autoState = AUTO_HALT; beep(2); }
        break;
      }

      if (elapsed >= AUTO_BACK_MAX_MS) {
        stopMotors();
        autoState = AUTO_HALT;
        beep(2);
        Serial.println("AUTO halt: no room behind");
      }
      break;
    }

    case AUTO_HALT:
    default:
      stopMotors();
      break;
  }

  showAutoStatus();
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

// 1.0 = left wheel untouched. The value is a multiplier, not an offset,
// so a motor that is 5% slow stays corrected at every speed.
void loadMotorBalance() {
  prefs.begin("maze", false);
  motorBalance = prefs.isKey("balance") ? prefs.getFloat("balance", 1.0f) : 1.0f;
  driveInvert = prefs.isKey("invert") ? prefs.getBool("invert", false) : true;
  prefs.end();
  motorBalance = constrain(motorBalance, 0.50f, 1.60f);
  Serial.printf("motor balance: %.3f\n", motorBalance);
  Serial.printf("drive invert: %s\n", driveInvert ? "on" : "off");
}

void saveDriveInvert() {
  prefs.begin("maze", false);
  prefs.putBool("invert", driveInvert);
  prefs.end();
}

void saveMotorBalance() {
  prefs.begin("maze", false);
  prefs.putFloat("balance", motorBalance);
  prefs.end();
  balanceDirty = false;
  lastBalanceSave = millis();
  Serial.printf("motor balance saved: %.3f\n", motorBalance);
}

// Five short straight bursts in open space. Each one measures the average yaw
// rate and nudges the left side until the robot stops turning on its own.
void calibrateMotorBalance() {
  stopMotors();
  readUltrasonic();
  if (distanceFront > 0 && distanceFront < 120) {
    if (voiceRec) voiceCancel();
    if (currentPage == VOICE_PAGE) deinitMic();
    currentPage = 4;
    showWrapped("TRIM", "need 1.2m clear ahead");
    return;
  }

  if (voiceRec) voiceCancel();
  if (currentPage == VOICE_PAGE) deinitMic();
  showWrapped("TRIM", "keep still...");
  calibrateGyroBias();
  beep(1);

  for (int run = 0; run < 5; run++) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.printf("TRIM RUN %d/5\n", run + 1);
    display.printf("L %+.1f%%\n", (motorBalance - 1.0f) * 100.0f);
    display.display();

    moveForward(130, 130);
    unsigned long start = millis();
    float yawSum = 0;
    int yawN = 0;
    while (millis() - start < 700) {
      sensors_event_t a, g, t;
      mpu.getEvent(&a, &g, &t);
      if (millis() - start > 150) {
        yawSum += (g.gyro.z * 180.0 / PI) - gyroZBiasDegPerSec;
        yawN++;
      }
      delay(20);
    }
    stopMotors();
    delay(300);

    float avgYaw = yawN ? (yawSum / yawN) : 0;
    float step = constrain(avgYaw * 0.0015f, -0.03f, 0.03f);
    motorBalance = constrain(motorBalance + step, 0.75f, 1.25f);
    Serial.printf("trim run %d: yaw %.1f -> %.3f\n", run + 1, avgYaw, motorBalance);
  }

  saveMotorBalance();
  currentPage = 4; // park on pose so nothing starts driving again
  showPosePage();
  beep(2);
  Serial.printf("trim done: L %+.1f%%\n", (motorBalance - 1.0f) * 100.0f);
}

// Rough translation speed from the last wheel command. The two speed
// constants were measured at SPEED_CAL_PWM, so a faster web/laptop burst
// doesn't pretend the robot is still crawling.
float driveSpeedEstimate() {
  if (driveState != DRIVE_FORWARD && driveState != DRIVE_BACKWARD) return 0;
  float avgPwm = (pwmLeft + pwmRight) * 0.5f;
  float speed = (driveState == DRIVE_FORWARD) ? FORWARD_SPEED_MM_S : -BACKWARD_SPEED_MM_S;
  return speed * (avgPwm / SPEED_CAL_PWM);
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
  lastAx = a.acceleration.x;
  lastAy = a.acceleration.y;
  lastAz = a.acceleration.z;
  lastGx = g.gyro.x * 180.0f / PI;
  lastGy = g.gyro.y * 180.0f / PI;
  lastGz = g.gyro.z * 180.0f / PI;
  float yawRateDegPerSec = (g.gyro.z * 180.0 / PI) - gyroZBiasDegPerSec;
  gyroYawRate = yawRateDegPerSec;
  pose.heading += yawRateDegPerSec * dt;
  if (pose.heading > 180) pose.heading -= 360;
  if (pose.heading < -180) pose.heading += 360;

  // Straight command + non-zero yaw = the wheels are not matched. Integrate
  // the error over a second and trim the left side a little. Slow on
  // purpose: a bump or a single bad gyro sample shouldn't move the value.
  if (straightCmd && driveState == DRIVE_FORWARD &&
      webHoldDir == 'S' && !straightTestActive && !wheelTestId) {
    trimYawSum += yawRateDegPerSec * dt;
    trimRunTime += dt;
    if (trimRunTime >= 1.0f) {
      float avgYaw = trimYawSum / trimRunTime;
      if (fabs(avgYaw) > 1.5f) {
        // Positive yaw = turning left (CCW), so the left wheel is the slow
        // one and its scale has to go up. Flipping the trim to the other
        // wheel means flipping this sign too.
        float step = constrain(avgYaw * 0.0015f, -0.025f, 0.025f);
        float nextBalance = constrain(motorBalance + step, 0.75f, 1.25f);
        if (fabs(nextBalance - motorBalance) > 0.0005f) {
          motorBalance = nextBalance;
          balanceDirty = true;
          Serial.printf("trim: yaw %.1f deg/s, balance %.3f\n", avgYaw, motorBalance);
        }
      }
      trimYawSum = 0;
      trimRunTime = 0;
    }
  } else {
    trimYawSum = 0;
    trimRunTime = 0;
  }
  if (balanceDirty && millis() - lastBalanceSave > 5000) saveMotorBalance();

  float speed = driveSpeedEstimate();

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
  float speed = driveSpeedEstimate();
  float headingRad = pose.heading * PI / 180.0f;
  predX = pose.x + speed * secondsAhead * cos(headingRad);
  predY = pose.y + speed * secondsAhead * sin(headingRad);
}

// -------------------- WiFi (STA + fallback AP, mDNS, OTA) --------------------
// Connecting happens in the background. setupWifi() never blocks: the web
// server is live on the fallback AP immediately, and the robot retries the
// phone hotspot every 20 s until it joins.
void setupWifi() {
  prefs.begin("maze", false);
  wifiSsid = prefs.isKey("ssid") ? prefs.getString("ssid", WIFI_SSID) : String(WIFI_SSID);
  wifiPass = prefs.isKey("pass") ? prefs.getString("pass", WIFI_PASS) : String(WIFI_PASS);
  prefs.end();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);
  apIP = WiFi.softAPIP().toString();
  botIP = apIP;
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  Serial.printf("AP '%s' ip %s\n", AP_SSID, apIP.c_str());

  // OTA and mDNS also work on the fallback AP, so the board can be updated
  // and controlled even when the phone hotspot is unavailable.
  if (MDNS.begin("mazerunner")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mdns: http://mazerunner.local");
  }
  ArduinoOTA.setHostname("mazerunner");
  ArduinoOTA.setPassword(OTA_PASS);
  ArduinoOTA.onStart([]() { stopMotors(); Serial.println("OTA start"); });
  ArduinoOTA.onEnd([]() { Serial.println("OTA end"); });
  ArduinoOTA.onError([](ota_error_t e) { Serial.printf("OTA error %u\n", (unsigned)e); });
  ArduinoOTA.begin();
  otaReady = true;
  Serial.println("ota ready (AP or STA)");

  if (wifiSsid.length() && wifiSsid != "YOUR_WIFI_NAME") {
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    Serial.printf("wifi: joining '%s'...\n", wifiSsid.c_str());
    wifiRetryAt = millis() + 20000;
  }
}

void wifiLoop() {
  static int lastStatus = -1;
  int st = (int)WiFi.status();
  if (st != lastStatus) {
    lastStatus = st;
    Serial.printf("wifi status %d\n", st);
  }

  if (st == WL_CONNECTED) {
    String ip = WiFi.localIP().toString();
    if (ip != botIP) {
      botIP = ip;
      Serial.printf("wifi ok: %s rssi %d\n", botIP.c_str(), WiFi.RSSI());
    }
    if (!netServicesStarted) {
      netServicesStarted = true;
      MDNS.begin("mazerunner");
      MDNS.addService("http", "tcp", 80);
      Serial.println("mdns refreshed for STA");
    }
  } else {
    if (netServicesStarted) {
      netServicesStarted = false;
      botIP = apIP;
      Serial.println("wifi lost, AP fallback");
    }
    if (wifiSsid.length() && millis() >= wifiRetryAt) {
      wifiRetryAt = millis() + 20000;
      Serial.printf("wifi retry '%s' (status %d)\n", wifiSsid.c_str(), st);
      WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    }
  }
}

// -------------------- Laptop link --------------------
// The heavy stuff (vosk + ollama) runs on the PC. The robot only posts
// sensor state or a wav file and waits for a short json answer.
void loadLaptopSettings() {
  prefs.begin("maze", false);
  String saved = prefs.isKey("laptop") ? prefs.getString("laptop", "") : "";
  prefs.end();
  if (saved.length()) laptopHost = saved;
  Serial.printf("laptop host: %s\n", laptopHost.length() ? laptopHost.c_str() : "(not set)");
}

String escJson(const String& s) {
  String o;
  o.reserve(s.length() + 8);
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"') o += "'";
    else if (c == '\\') o += '/';
    else if (c == '\n' || c == '\r' || c == '\t') o += ' ';
    else if ((unsigned char)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}

String jsonField(const String& js, const char* key) {
  String pat = String("\"") + key + "\"";
  int n = js.length();
  int k = -1, from = 0;
  // Find the key itself, not a copy of the name inside some other string.
  while (true) {
    k = js.indexOf(pat, from);
    if (k < 0) return "";
    int c = k + pat.length();
    while (c < n && (js[c] == ' ' || js[c] == '\t' || js[c] == '\r' || js[c] == '\n')) c++;
    if (c < n && js[c] == ':') break;
    from = k + 1;
  }
  int i = js.indexOf(':', k + pat.length());
  if (i < 0) return "";
  i++;
  while (i < n && (js[i] == ' ' || js[i] == '\t' || js[i] == '\r' || js[i] == '\n')) i++;
  if (i >= n) return "";

  // String value: copy until the closing quote, skipping escapes.
  if (js[i] == '"') {
    int e = i + 1;
    while (e < n) {
      if (js[e] == '\\') { e += 2; continue; }
      if (js[e] == '"') break;
      e++;
    }
    return js.substring(i + 1, e);
  }

  // Number/bool/null: stop at the next structural character.
  int e = i;
  while (e < n && js[e] != ',' && js[e] != '}' && js[e] != ']') e++;
  String v = js.substring(i, e);
  v.trim();
  return v;
}

String httpPostJson(const String& url, const String& body, int* code) {
  HTTPClient http;
  http.setConnectTimeout(4000);
  http.setTimeout(20000);
  if (!http.begin(url)) {
    if (code) *code = -1;
    return "";
  }
  http.addHeader("Content-Type", "application/json");
  int c = http.POST((uint8_t*)body.c_str(), body.length());
  String resp = (c > 0) ? http.getString() : "";
  http.end();
  if (code) *code = c;
  Serial.printf("POST %s -> %d\n", url.c_str(), c);
  return resp;
}

String httpPostWav(const String& url, const uint8_t* data, size_t len, int* code) {
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(30000);
  if (!http.begin(url)) {
    if (code) *code = -1;
    return "";
  }
  http.addHeader("Content-Type", "audio/wav");
  int c = http.POST((uint8_t*)data, len);
  String resp = (c > 0) ? http.getString() : "";
  http.end();
  if (code) *code = c;
  Serial.printf("POST wav %u bytes -> %d\n", (unsigned)len, c);
  return resp;
}

void showLaptopPage() {
  String msg = "IP: ";
  msg += laptopHost.length() ? laptopHost : "set it on the web page";
  msg += "\n";
  msg += lastLaptopAction;
  if (lastLaptopThought.length()) {
    msg += "\n";
    msg += lastLaptopThought;
  }
  showWrapped("LAPTOP BRAIN", msg);
}

// Asks the laptop for one action every 2s. Ollama is slow, so the robot
// moves in short bursts and stops in between. Local front distance still
// overrides a forward command, no matter what the model says.
void laptopDrive() {
  static unsigned long last = 0;
  if (millis() - last < 2000) return;
  last = millis();

  distLeft = sensorLeft.readRangeContinuousMillimeters();
  distRight = sensorRight.readRangeContinuousMillimeters();
  if (sensorLeft.timeoutOccurred()) distLeft = 65535;
  if (sensorRight.timeoutOccurred()) distRight = 65535;
  readUltrasonic();

  if (!laptopHost.length()) {
    stopMotors();
    lastLaptopAction = "no laptop ip";
    lastLaptopThought = "open the web page and set it";
    showLaptopPage();
    return;
  }

  String body = "{\"front\":";
  body += String(distanceFront > 0 ? distanceFront * 10.0f : 999);
  body += ",\"left\":";
  body += String(distLeft == 65535 ? 999 : distLeft);
  body += ",\"right\":";
  body += String(distRight == 65535 ? 999 : distRight);
  body += ",\"heading\":";
  body += String(pose.heading, 1);
  body += "}";

  int code = 0;
  String resp = httpPostJson("http://" + laptopHost + ":5000/drive", body, &code);
  if (code <= 0 || code >= 400) {
    stopMotors();
    lastLaptopAction = code ? "laptop error" : "laptop offline";
    lastLaptopThought = code ? (String("http ") + String(code)) : String("run server.py on the pc");
    showLaptopPage();
    return;
  }

  String action = jsonField(resp, "action");
  action.trim();
  action.toLowerCase();
  int speed = jsonField(resp, "speed").toInt();
  if (speed < 60 || speed > 220) speed = 140;
  String thought = jsonField(resp, "thought");
  if (thought.length() > 60) thought = thought.substring(0, 60);

  bool frontClose = (distanceFront > 0 && distanceFront < 15);
  if (action == "forward" && !frontClose) {
    moveForward(speed, speed);
    delay(500);
  } else if (action == "backward") {
    moveBackward(speed, speed);
    delay(500);
  } else if (action == "left") {
    spinCCW(speed, 400);
  } else if (action == "right") {
    spinCW(speed, 400);
  }
  stopMotors();

  lastLaptopAction = action.length() ? action : "no action";
  lastLaptopThought = thought;
  showLaptopPage();
}

// -------------------- Control website --------------------
String escHtml(const String& s) {
  String o = s;
  o.replace("&", "&amp;");
  o.replace("<", "&lt;");
  o.replace(">", "&gt;");
  o.replace("\"", "&quot;");
  o.replace("'", "&#39;");
  return o;
}

static const char HTML_PAGE[] PROGMEM = R"HTML(<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>MazeRunner</title>
<style>
:root{--bg:#0b0f14;--card:#151c24;--line:#27313d;--fg:#e7edf3;--dim:#8fa0b3;--accent:#4cc2ff;--go:#2ecc71;--stop:#ff5252}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{margin:0 auto;max-width:760px;padding:16px;background:var(--bg);color:var(--fg);font-family:system-ui,"Segoe UI",Roboto,sans-serif}
h1{font-size:20px;letter-spacing:.22em;margin:0 0 4px}
#status{font-size:12px;color:var(--dim);margin-bottom:14px}
#dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#ff5252;margin-right:6px}
#dot.on{background:var(--go);box-shadow:0 0 8px var(--go)}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
.tile{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}
.k{color:var(--dim);font-size:11px;letter-spacing:.12em}
.v{font-size:26px;font-variant-numeric:tabular-nums;margin-top:2px}
.u{font-size:12px;color:var(--dim)}
.wide{margin-top:10px}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin:14px 0}
button{border:1px solid var(--line);background:var(--card);color:var(--fg);border-radius:12px;padding:16px;font-size:18px;font-weight:600;touch-action:none;user-select:none;cursor:pointer}
button:active,button.held{background:#203040;border-color:var(--accent)}
button.stop{background:#2a1517;border-color:#7a2a2e;color:#ff8b8b}
button.auto{background:#10251a;border-color:#245c3a;color:#7ef0ad}
button.auto.stop{background:#2a1517;border-color:#7a2a2e;color:#ff8b8b}
.row{display:flex;gap:8px;margin-top:10px}
.row button{flex:1;padding:13px;font-size:14px}
.foot{color:var(--dim);font-size:11px;margin-top:16px;line-height:1.5}
input{width:100%;margin:5px 0;padding:11px;border-radius:9px;border:1px solid var(--line);background:#0f151c;color:var(--fg);font-size:15px}
.diag{display:grid;grid-template-columns:1fr 1fr;gap:7px 14px;margin-top:8px;font-size:13px}
.diag div{display:flex;justify-content:space-between;gap:8px;border-bottom:1px solid #1d2530;padding-bottom:4px}
.diag span{color:var(--dim)}
.diag b{font-weight:600;font-variant-numeric:tabular-nums;text-align:right}
.row4{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:10px}
.row4 button{padding:11px 4px;font-size:12px}
code{color:var(--accent)}
</style></head><body>
<h1>MAZERUNNER</h1>
<div id="status"><span id="dot"></span><span id="link">connecting...</span></div>

<div class="grid">
  <div class="tile"><div class="k">FRONT</div><div><span class="v" id="front">--</span><span class="u"> cm</span></div></div>
  <div class="tile"><div class="k">LEFT</div><div><span class="v" id="left">--</span><span class="u"> mm</span></div></div>
  <div class="tile"><div class="k">RIGHT</div><div><span class="v" id="right">--</span><span class="u"> mm</span></div></div>
  <div class="tile"><div class="k">HEADING</div><div><span class="v" id="heading">--</span><span class="u"> deg</span></div></div>
</div>

<div class="tile wide"><div class="k">MODE</div><div class="v" id="mode" style="font-size:18px">manual</div>
<div class="u" id="meta">trim --</div></div>

<div class="tile wide"><div class="k">LIVE SENSOR DIAGNOSTICS</div>
<div class="diag">
  <div><span>front echo</span><b id="d_fus">--</b></div>
  <div><span>front state</span><b id="d_fok">--</b></div>
  <div><span>left ToF</span><b id="d_lok">--</b></div>
  <div><span>right ToF</span><b id="d_rok">--</b></div>
  <div><span>yaw rate</span><b id="d_yaw">--</b></div>
  <div><span>gyro bias</span><b id="d_bias">--</b></div>
  <div><span>accel x/y/z</span><b id="d_acc">--</b></div>
  <div><span>gyro x/y/z</span><b id="d_gyr">--</b></div>
  <div><span>wheel PWM</span><b id="d_pwm">--</b></div>
  <div><span>drive state</span><b id="d_drive">--</b></div>
  <div><span>test</span><b id="d_test">--</b></div>
  <div><span>wifi</span><b id="d_wifi">--</b></div>
</div></div>

<div class="pad">
  <div></div><button id="f">&#9650;</button><div></div>
  <button id="l">&#9664;</button><button id="s" class="stop">STOP</button><button id="r">&#9654;</button>
  <div></div><button id="b">&#9660;</button><div></div>
</div>

<div class="row">
  <button id="auto" class="auto">AUTO START</button>
  <button id="autostop" class="auto stop">AUTO STOP</button>
</div>
<div class="row">
  <button id="tm">TRIM -</button>
  <button id="tp">TRIM +</button>
  <button id="tr">TRIM RESET</button>
</div>
<div class="row">
  <button id="inv">SWAP F/B</button>
</div>

<div class="tile wide"><div class="k">INDEPENDENT WHEEL TESTS (0.9 s)</div>
<div class="row4">
  <button data-wheel="1">L fwd</button>
  <button data-wheel="2">L rev</button>
  <button data-wheel="3">R fwd</button>
  <button data-wheel="4">R rev</button>
</div>
<div class="row4">
  <button data-both="f">BOTH fwd</button>
  <button data-both="b">BOTH rev</button>
  <button id="wstop" class="stop">STOP</button>
</div>
<div class="u" style="margin-top:8px">Each wheel runs alone. Watch which wheel spins and which way; the OLED names the active test.</div>
</div>

<div class="tile wide"><div class="k">WIFI</div>
<div class="u" id="wifi">not connected</div>
<form action="/wifi" method="post" style="margin-top:8px">
  <input id="ssid" name="ssid" placeholder="network name (SSID)" autocomplete="off" autocapitalize="none" spellcheck="false">
  <input id="pass" name="pass" type="password" placeholder="password" autocomplete="off">
  <button type="submit" class="auto" style="width:100%">SAVE &amp; CONNECT</button>
</form>
<div class="row">
  <button id="wretry">RETRY WIFI</button>
  <button id="reboot" class="stop">REBOOT</button>
</div>
<div class="u" style="margin-top:8px">The robot joins this network in the background and stays on the fallback AP until it succeeds.</div>
</div>

<div class="foot">
Hold a direction button to drive, release to stop. The robot also stops by itself
if the page goes offline for 8 s.<br>
AP fallback: join <code>MazeRunner</code> / <code>mazerunner123</code>, then open
<code>http://192.168.4.1</code>.
</div>

<script>
const q=id=>document.getElementById(id);
let held=null, keep=null;
const send=(p)=>fetch(p,{cache:'no-store'}).catch(()=>{});
function stopAll(){held=null;clearInterval(keep);send('/drive?dir=S');}
function bindHold(id,dir){
  const b=q(id);
  const down=e=>{e.preventDefault();held=dir;b.classList.add('held');send('/drive?dir='+dir);
    clearInterval(keep);keep=setInterval(()=>{if(held===dir)send('/drive?dir='+dir);},1500);};
  const up=e=>{e.preventDefault();if(held===dir)stopAll();b.classList.remove('held');};
  b.addEventListener('pointerdown',down);
  b.addEventListener('pointerup',up);
  b.addEventListener('pointerleave',up);
  b.addEventListener('pointercancel',up);
  b.addEventListener('contextmenu',e=>e.preventDefault());
}
bindHold('f','F');bindHold('b','B');bindHold('l','L');bindHold('r','R');
q('s').addEventListener('pointerdown',e=>{e.preventDefault();stopAll();});
q('auto').addEventListener('pointerdown',e=>{e.preventDefault();stopAll();send('/auto?on=1');});
q('autostop').addEventListener('pointerdown',e=>{e.preventDefault();stopAll();send('/auto?on=0');});
q('tm').onclick=()=>send('/trim?step=-5');
q('tp').onclick=()=>send('/trim?step=5');
q('tr').onclick=()=>send('/trim?reset=1');
q('inv').onclick=()=>send('/invert');
document.querySelectorAll('[data-wheel]').forEach(b=>b.onclick=()=>send('/test?wheel='+b.dataset.wheel));
document.querySelectorAll('[data-both]').forEach(b=>b.onclick=()=>send('/test?both='+b.dataset.both));
q('wstop').onclick=()=>stopAll();
q('wretry').onclick=()=>send('/wifi?retry=1');
q('reboot').onclick=()=>{if(confirm('Reboot the robot?'))send('/reboot');};
window.addEventListener('blur',stopAll);
window.addEventListener('pagehide',()=>{navigator.sendBeacon('/drive?dir=S');});
async function poll(){
  try{
    const r=await fetch('/sensors',{cache:'no-store'});
    const j=await r.json();
    q('dot').classList.add('on');
    q('link').textContent=j.wifi_connected
      ? ('online  '+j.ip+'  '+j.rssi+' dBm')
      : ('online  AP only  '+j.ap_ip);
    q('front').textContent=j.front>0?Math.round(j.front):'out';
    q('left').textContent=j.left===65535?'out':j.left;
    q('right').textContent=j.right===65535?'out':j.right;
    q('heading').textContent=(j.heading>=0?'+':'')+Math.round(j.heading);
    q('mode').textContent=j.auto?'AUTO '+j.state:'manual';
    q('meta').textContent='trim '+j.trim.toFixed(1)+'%   speed '+j.speed+'   '+j.ip;
    q('inv').textContent='SWAP F/B  ('+(j.inv?'swapped':'normal')+')';
    q('wifi').textContent='network: '+j.ssid+'   address: '+j.ip;
    q('d_fus').textContent=j.front_ok?j.front_us+' us':'no echo';
    q('d_fok').textContent=j.front_ok?'OK':'OUT';
    q('d_lok').textContent=j.left_ok?(j.left+' mm'):'TIMEOUT';
    q('d_rok').textContent=j.right_ok?(j.right+' mm'):'TIMEOUT';
    q('d_yaw').textContent=j.yaw.toFixed(1)+' deg/s';
    q('d_bias').textContent=j.bias.toFixed(2)+' deg/s';
    q('d_acc').textContent=j.ax.toFixed(2)+' '+j.ay.toFixed(2)+' '+j.az.toFixed(2);
    q('d_gyr').textContent=j.gx.toFixed(1)+' '+j.gy.toFixed(1)+' '+j.gz.toFixed(1);
    q('d_pwm').textContent=j.pwm_l+' / '+j.pwm_r;
    q('d_drive').textContent=j.drive;
    q('d_test').textContent=j.wheel_test?('wheel '+j.wheel_test):(j.straight?'straight':'idle');
    q('d_wifi').textContent=j.wifi_connected?('STA '+j.rssi+' dBm'):('AP '+j.ap_ip);
  }catch(e){
    q('dot').classList.remove('on');
    q('link').textContent='offline - robot not reachable';
  }
}
setInterval(poll,300);poll();
</script>
</body></html>)HTML";

void handleWebRoot() {
  webServer.send_P(200, "text/html", HTML_PAGE);
}

void webBurst(const String& dir) {
  if (dir == "F") { moveForward(webSpeed, webSpeed); delay(600); }
  else if (dir == "B") { moveBackward(webSpeed, webSpeed); delay(600); }
  else if (dir == "L") { spinCCW(webSpeed, 400); }
  else if (dir == "R") { spinCW(webSpeed, 400); }
  stopMotors();
}

// Hold-to-drive: the page sends a direction on pointer-down and S on
// pointer-up. Applying it every loop keeps the motors moving without any
// blocking delay in the web handler. An 8 s watchdog stops a lost connection.
void webDriveApply() {
  if (webHoldDir == 'S') return;
  if (millis() - webHoldLast > 8000) {
    webHoldDir = 'S';
    stopMotors();
    return;
  }
  switch (webHoldDir) {
    case 'F': moveForward(webSpeed, webSpeed); break;
    case 'B': moveBackward(webSpeed, webSpeed); break;
    case 'L': { int s = min(webSpeed, 120); setWheels(-s, s); break; }
    case 'R': { int s = min(webSpeed, 120); setWheels(s, -s); break; }
    default: webHoldDir = 'S'; stopMotors(); break;
  }
}

void handleWebDrive() {
  String d = webServer.arg("dir");
  if (!d.length()) d = "S";
  char c = toupper(d[0]);
  if (c == 'S') {
    webHoldDir = 'S';
    wheelTestId = 0;
    straightTestActive = false;
    stopMotors();
  } else if (c == 'F' || c == 'B' || c == 'L' || c == 'R') {
    webHoldDir = c;
    webHoldLast = millis();
    wheelTestId = 0;
    straightTestActive = false;
    if (currentPage == 3) autoState = AUTO_HALT;
    currentPage = 0;
    webDriveApply();
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleWebAuto() {
  bool on = webServer.arg("on") == "1";
  webHoldDir = 'S';
  wheelTestId = 0;
  straightTestActive = false;
  stopMotors();
  if (on) {
    currentPage = 3;
    autoReset();
  } else {
    autoState = AUTO_HALT;
    currentPage = 0;
    showWelcomePage();
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleWebInvert() {
  if (webServer.hasArg("on")) driveInvert = webServer.arg("on") == "1";
  else driveInvert = !driveInvert;
  saveDriveInvert();
  Serial.printf("drive invert: %s\n", driveInvert ? "on" : "off");
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleWebWifi() {
  String s = webServer.arg("ssid");
  s.trim();
  String p = webServer.arg("pass");
  if (s.length()) {
    wifiSsid = s;
    wifiPass = p;
    prefs.begin("maze", false);
    prefs.putString("ssid", wifiSsid);
    prefs.putString("pass", wifiPass);
    prefs.end();
    WiFi.disconnect();
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    wifiRetryAt = millis() + 20000;
    Serial.printf("wifi config: '%s'\n", wifiSsid.c_str());
  } else if (webServer.hasArg("retry")) {
    WiFi.disconnect();
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
    wifiRetryAt = millis() + 20000;
    Serial.printf("wifi retry button: '%s'\n", wifiSsid.c_str());
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleWebTest() {
  webHoldDir = 'S';
  wheelTestId = 0;
  straightTestActive = false;
  stopMotors();
  int id = webServer.arg("wheel").toInt();
  if (id >= 1 && id <= 4) {
    startWheelTest(id);
  } else {
    String both = webServer.arg("both");
    if (both == "f") startStraightTest(130, true);
    else if (both == "b") startStraightTest(130, false);
  }
  webServer.send(200, "application/json", "{\"ok\":true}");
}

void handleWebReboot() {
  webHoldDir = 'S';
  stopMotors();
  webServer.send(200, "application/json", "{\"reboot\":true}");
  delay(200);
  ESP.restart();
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

void handleWebTrim() {
  if (webServer.hasArg("cal")) {
    calibrateMotorBalance();
  } else if (webServer.hasArg("reset")) {
    motorBalance = 1.0f;
    saveMotorBalance();
  } else if (webServer.hasArg("step")) {
    float pct = (motorBalance - 1.0f) * 100.0f + webServer.arg("step").toFloat();
    motorBalance = constrain(1.0f + pct / 100.0f, 0.50f, 1.60f);
    saveMotorBalance();
  } else if (webServer.hasArg("b")) {
    float pct = webServer.arg("b").toFloat();
    motorBalance = constrain(1.0f + pct / 100.0f, 0.50f, 1.60f);
    saveMotorBalance();
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleWebAsk() {
  String q = webServer.arg("q");
  q.trim();
  lastAIAnswer = "";
  if (!q.length()) {
    lastAIAnswer = "type a question first";
  } else if (!laptopHost.length()) {
    lastAIAnswer = "set the laptop ip first";
  } else {
    int code = 0;
    String resp = httpPostJson("http://" + laptopHost + ":5000/ask",
                               "{\"question\":\"" + escJson(q) + "\"}", &code);
    if (code <= 0) lastAIAnswer = "laptop offline";
    else if (code >= 400) lastAIAnswer = String("laptop error ") + code;
    else {
      lastAIAnswer = jsonField(resp, "response");
      if (!lastAIAnswer.length()) lastAIAnswer = "empty reply";
    }
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

void handleWebTarget() {
  String host = webServer.arg("host");
  host.trim();
  if (host.length()) {
    laptopHost = host;
    prefs.begin("maze", false);
    prefs.putString("laptop", laptopHost);
    prefs.end();
    Serial.printf("laptop host set to %s\n", laptopHost.c_str());
  }
  webServer.sendHeader("Location", "/");
  webServer.send(303);
}

String buildTelemetryJson() {
  uint16_t l = sensorLeft.readRangeContinuousMillimeters();
  bool lt = sensorLeft.timeoutOccurred();
  uint16_t r = sensorRight.readRangeContinuousMillimeters();
  bool rt = sensorRight.timeoutOccurred();
  if (lt) l = 65535;
  if (rt) r = 65535;
  readUltrasonic();

  String j = "{";
  j += "\"ms\":" + String(millis()) + ",";
  j += "\"front\":" + String(distanceFront, 1) + ",";
  j += "\"front_us\":" + String(frontEchoUs) + ",";
  j += "\"front_ok\":" + String(frontEchoValid ? "true" : "false") + ",";
  j += "\"left\":" + String(l) + ",\"left_ok\":" + String(lt ? "false" : "true") + ",";
  j += "\"right\":" + String(r) + ",\"right_ok\":" + String(rt ? "false" : "true") + ",";
  j += "\"heading\":" + String(pose.heading, 1) + ",";
  j += "\"yaw\":" + String(gyroYawRate, 1) + ",";
  j += "\"bias\":" + String(gyroZBiasDegPerSec, 2) + ",";
  j += "\"ax\":" + String(lastAx, 2) + ",\"ay\":" + String(lastAy, 2) +
       ",\"az\":" + String(lastAz, 2) + ",";
  j += "\"gx\":" + String(lastGx, 1) + ",\"gy\":" + String(lastGy, 1) +
       ",\"gz\":" + String(lastGz, 1) + ",";
  j += "\"pwm_l\":" + String(pwmLeft) + ",\"pwm_r\":" + String(pwmRight) + ",";
  j += "\"drive\":\"" + String(driveStateName()) + "\",";
  j += "\"trim\":" + String((motorBalance - 1.0f) * 100.0f, 2) + ",";
  j += "\"speed\":" + String(webSpeed) + ",";
  j += "\"inv\":" + String(driveInvert ? "true" : "false") + ",";
  j += "\"page\":" + String(currentPage) + ",";
  j += "\"auto\":" + String(currentPage == 3 ? "true" : "false") + ",";
  j += "\"state\":\"" + String(autoStateName()) + "\",";
  j += "\"wheel_test\":" + String(wheelTestId) + ",";
  j += "\"straight\":" + String(straightTestActive ? "true" : "false") + ",";
  j += "\"ssid\":\"" + wifiSsid + "\",";
  j += "\"ip\":\"" + botIP + "\",";
  j += "\"ap_ip\":\"" + apIP + "\",";
  j += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  j += "\"wifi_status\":" + String((int)WiFi.status()) + ",";
  j += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  j += "\"uptime\":" + String(millis() / 1000) + "}";
  return j;
}

void handleWebSensors() {
  webServer.send(200, "application/json", buildTelemetryJson());
}

void setupWeb() {
  webServer.on("/", handleWebRoot);
  webServer.on("/cmd", handleWebCmd);
  webServer.on("/drive", handleWebDrive);
  webServer.on("/auto", handleWebAuto);
  webServer.on("/invert", handleWebInvert);
  webServer.on("/wifi", handleWebWifi);
  webServer.on("/test", handleWebTest);
  webServer.on("/reboot", handleWebReboot);
  webServer.on("/speed", handleWebSpeed);
  webServer.on("/trim", handleWebTrim);
  webServer.on("/sensors", handleWebSensors);
  webServer.on("/ask", handleWebAsk);
  webServer.on("/target", handleWebTarget);
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
  if (i2s_set_pin(I2S_PORT, &pins) != ESP_OK) {
    i2s_driver_uninstall(I2S_PORT);
    return false;
  }
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
    showWrapped("VOICE", laptopHost.length()
                             ? "thinking... (laptop transcribes + asks ollama)"
                             : "thinking... (PC transcribes + asks ollama)");
    return;
  }
  if (voiceStatus == V_NOLINK) {
    showWrapped("VOICE", laptopHost.length()
                             ? "no answer from laptop. is server.py running?"
                             : "no reply. start mazerunner_voice.py on the PC.");
    return;
  }
  if (voiceStatus == V_RESULT) {
    String both = voiceHeard;
    if (voiceReply.length()) both += String("\nAI: ") + voiceReply;
    showWrapped("YOU SAID", both);
    return;
  }
  // idle
  String msg = "Hold BTN2, speak, release.\n";
  msg += laptopHost.length() ? ("Laptop: " + laptopHost) : String("USB: mazerunner_voice.py");
  msg += "\nIP: " + botIP;
  if (lastVoiceText.length()) msg += String("\nLast: ") + lastVoiceText;
  showWrapped("VOICE", msg);
}

void voiceStart() {
  if (!micOk) return;
  // keep wifi uploads short, 5s is plenty for one sentence
  size_t secs = laptopHost.length() ? 5 : voiceMaxSec();
  size_t maxSamples = secs * I2S_SAMPLE_RATE;
  if (voiceBufAlloc) { free(voiceBufAlloc); voiceBufAlloc = nullptr; voiceBuf = nullptr; }
  voiceBufAlloc = (uint8_t *)malloc(44 + maxSamples * sizeof(int16_t));
  if (!voiceBufAlloc) {
    voiceStatus = V_IDLE;
    showWrapped("VOICE", "out of memory for recording");
    return;
  }
  voiceBuf = (int16_t *)(voiceBufAlloc + 44);
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
  if (voiceBufAlloc) { free(voiceBufAlloc); voiceBufAlloc = nullptr; voiceBuf = nullptr; }
  voiceCap = voiceLen = 0;
  voiceStatus = V_IDLE;
}

void voiceFinishSend() {
  voiceRec = false;
  uint32_t n = (uint32_t)(voiceLen * sizeof(int16_t));
  lastVoiceText = "";
  if (laptopHost.length()) {
    voiceSendWifi();
    return;
  }
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
  if (voiceBufAlloc) { free(voiceBufAlloc); voiceBufAlloc = nullptr; voiceBuf = nullptr; }
  voiceCap = voiceLen = 0;
  voiceStatus = V_WAITING;
  voiceWaitStart = millis();
  showVoicePage();
}

// -------------------- Voice over wifi --------------------
static void put16(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void put32(uint8_t *p, uint32_t v) {
  p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = v >> 24;
}

static void wavHeader(uint8_t *h, uint32_t dataLen) {
  memcpy(h, "RIFF", 4);
  put32(h + 4, 36 + dataLen);
  memcpy(h + 8, "WAVEfmt ", 8);
  put32(h + 16, 16);
  put16(h + 20, 1);                  // pcm
  put16(h + 22, 1);                  // mono
  put32(h + 24, I2S_SAMPLE_RATE);
  put32(h + 28, I2S_SAMPLE_RATE * 2);
  put16(h + 32, 2);                  // block align
  put16(h + 34, 16);                 // bits
  memcpy(h + 36, "data", 4);
  put32(h + 40, dataLen);
}

void voiceSendWifi() {
  uint32_t dataLen = (uint32_t)(voiceLen * sizeof(int16_t));
  wavHeader(voiceBufAlloc, dataLen);
  voiceStatus = V_WAITING;
  voiceWaitStart = millis();
  showVoicePage();

  int code = 0;
  String resp = httpPostWav("http://" + laptopHost + ":5000/stt",
                            voiceBufAlloc, 44 + dataLen, &code);
  if (voiceBufAlloc) { free(voiceBufAlloc); voiceBufAlloc = nullptr; voiceBuf = nullptr; }
  voiceCap = voiceLen = 0;

  if (code <= 0 || code >= 400) {
    voiceStatus = V_NOLINK;
    Serial.printf("voice wifi failed, code %d\n", code);
    showVoicePage();
    return;
  }
  voiceHeard = jsonField(resp, "text");
  voiceReply = jsonField(resp, "reply");
  if (!voiceHeard.length()) voiceHeard = "(nothing heard)";
  lastVoiceText = voiceHeard + (voiceReply.length() ? " -> " + voiceReply : "");
  voiceStatus = V_RESULT;
  showVoicePage();
}

// PC answers "TXTHDR" + uint32le len + utf8 ("YOU: ...\nAI: ..."). Runs every loop.
// Tiny USB console for bench testing. Send !P for one sensor sample, !A to
// start the autonomous page, !X to stop, !C to re-zero the gyro, !0 to reset
// the wheel trim, !T<percent> to set it (e.g. !T109 = 1.09), !S for a 2 s
// straight-line test at fixed PWM, and !L/!R/!F/!B for short movement bursts.
// The robot still has to be on a clear patch.
void handleSerialCmd(char c) {
  switch (c) {
    case 'P': case 'p': {
      uint16_t l = sensorLeft.readRangeContinuousMillimeters();
      bool lt = sensorLeft.timeoutOccurred();
      uint16_t r = sensorRight.readRangeContinuousMillimeters();
      bool rt = sensorRight.timeoutOccurred();
      readUltrasonic();
      Serial.printf("SENS F=%.1fcm L=%u%s R=%u%s H=%.1f BAL=%.3f INV=%d PAGE=%u\n",
                    distanceFront, l, lt ? "!" : "", r, rt ? "!" : "",
                    pose.heading, motorBalance, driveInvert ? 1 : 0, currentPage);
      break;
    }
    case 'A': case 'a':
      stopMotors();
      currentPage = 3;
      autoReset();
      Serial.println("AUTO start");
      break;
    case 'X': case 'x':
      stopMotors();
      currentPage = 0;
      showWelcomePage();
      Serial.println("stop");
      break;
    case 'C': case 'c':
      stopMotors();
      calibrateGyroBias();
      Serial.println("gyro calibrated");
      break;
    case '0':
      motorBalance = 1.0f;
      saveMotorBalance();
      Serial.println("trim reset: 1.000");
      break;
    case 'S': case 's':
      startStraightTest(130, true);
      break;
    case 'I': case 'i':
      driveInvert = !driveInvert;
      saveDriveInvert();
      Serial.printf("drive invert: %s\n", driveInvert ? "on" : "off");
      break;
    case '1': case '2': case '3': case '4':
      startWheelTest(c - '0');
      break;
    case 'J': case 'j':
      Serial.println(buildTelemetryJson());
      break;
    case 'W': case 'w': {
      wifiRetryAt = millis() + 60000;
      Serial.println("wifi scan: STA-only");
      WiFi.disconnect(true);
      WiFi.mode(WIFI_STA);
      delay(400);
      Serial.printf("scan mode %d status %d\n", (int)WiFi.getMode(), (int)WiFi.status());
      int n = WiFi.scanNetworks(false, true);
      Serial.printf("scan found %d\n", n);
      for (int i = 0; i < n && i < 20; i++) {
        Serial.printf("  %d: '%s' rssi %d ch %d auth %d\n", i,
                      WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i),
                      (int)WiFi.encryptionType(i));
      }
      WiFi.scanDelete();
      WiFi.mode(WIFI_AP_STA);
      WiFi.softAP(AP_SSID, AP_PASS);
      WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
      wifiRetryAt = millis() + 20000;
      break;
    }
    case 'L': case 'l':
      stopMotors();
      spinCCW(110, 300);
      Serial.printf("TEST CCW H=%.1f\n", pose.heading);
      break;
    case 'R': case 'r':
      stopMotors();
      spinCW(110, 300);
      Serial.printf("TEST CW H=%.1f\n", pose.heading);
      break;
    case 'F': case 'f':
      moveForward(130, 130);
      delay(500);
      brakeMotors(30);
      Serial.println("TEST FWD");
      break;
    case 'B': case 'b':
      moveBackward(130, 130);
      delay(500);
      brakeMotors(30);
      Serial.println("TEST BACK");
      break;
    default:
      break;
  }
}

// Manual wheel trim from the serial console. The gyro learner needs a long
// straight run and auto steers every cycle, so bench tuning sets it by hand.
void setTrimPercent(int pct) {
  pct = constrain(pct, 50, 160);
  motorBalance = pct / 100.0f;
  saveMotorBalance();
  Serial.printf("trim set: %.3f\n", motorBalance);
}

void pollSerialReply() {
  static uint8_t stage = 0; // 0 = magic, 1 = len, 2 = payload
  static bool cmd = false;
  static bool trimNum = false;
  static String trimDigits = "";
  static uint8_t mi = 0, li = 0;
  static uint32_t tlen = 0;
  static String payload = "";
  const char *magic = "TXTHDR";
  while (Serial.available()) {
    uint8_t c = Serial.read();
    if (stage == 0) {
      if (trimNum) {
        // !T<digits> ends at the first non-digit, normally a newline. The
        // same byte may start the next command, so re-arm on '!'.
        if (c >= '0' && c <= '9') { trimDigits += (char)c; continue; }
        if (trimDigits.length()) setTrimPercent(trimDigits.toInt());
        trimNum = false;
        trimDigits = "";
        if (c == '!') cmd = true;
        continue;
      }
      if (cmd) {
        cmd = false;
        if (c == 'T' || c == 't') { trimNum = true; trimDigits = ""; }
        else handleSerialCmd((char)c);
        continue;
      }
      if (c == '!') { cmd = true; continue; }
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
