/*
 * ESP32-S3 Firmware: Dynamic Step Pulse Engine for 1/32 Microstepping NEMA 17
 * -----------------------------------------------------------------------------
 * Listens for UDP: "CTRL <signed_speed_steps_per_sec> <tilt_deg>"
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ld2410.h>
#include <ESP32Servo.h>

const char* WIFI_SSID     = "Raspberry-π";
const char* WIFI_PASSWORD = "tore dimuna";

IPAddress piIP(192, 168, 0, 189);

const uint16_t PI_PORT    = 5005;
const uint16_t CMD_PORT   = 5007;
const uint16_t DEBUG_PORT = 5006;

const float ALERT_DISTANCE_CM = 50.0;
const unsigned long READ_INTERVAL_MS = 100;

#define STEP_PIN  8
#define DIR_PIN   9
#define SERVO_PIN 10

const int PAN_DIRECTION = 1;                  // Set to 1 or -1 to invert motor direction
const unsigned int STEP_PULSE_US = 3;         // 3us pulse width for 1/32 driver timing

const int SERVO_MIN_ANGLE = 40;
const int SERVO_MAX_ANGLE = 120;
const unsigned long SERVO_STEP_DELAY_MS = 5;  // 5ms step delay for smooth tilt tracking

const unsigned long CTRL_TIMEOUT_MS = 800;    // Motor timeout if Pi connection stops

#define R1_RX 4
#define R1_TX 5
#define R2_RX 6
#define R2_TX 7
#define R3_RX 15
#define R3_TX 16

HardwareSerial SerialR1(1);
HardwareSerial SerialR2(2);
HardwareSerial SerialR3(0);
ld2410 radar1, radar2, radar3;

WiFiUDP udp;
Servo tiltServo;

// Proportional Motion Engine State
float currentPanSpeed = 0.0;                  // Target step frequency (steps/sec)
int currentPanDir = 0;
int servoCurrent = SERVO_MIN_ANGLE;
int servoTarget  = SERVO_MIN_ANGLE;

unsigned long lastServoStepMs = 0;
unsigned long lastStepUs = 0;
unsigned long lastCtrlMs = 0;

void sendTo(const IPAddress &ip, uint16_t port, const String &payload) {
  udp.beginPacket(ip, port);
  udp.print(payload);
  udp.endPacket();
}

void debugLog(const String &msg) { sendTo(piIP, DEBUG_PORT, msg); }
void sendToPi(const String &payload) { sendTo(piIP, PI_PORT, payload); }

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  uint8_t tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(250);
    tries++;
  }
}

void setPanDirection(int dir) {
  if (dir == currentPanDir) return;
  currentPanDir = dir;
  digitalWrite(DIR_PIN, (dir * PAN_DIRECTION > 0) ? HIGH : LOW);
  delayMicroseconds(20);
}

// Proportional Stepper Engine
void handlePan() {
  float absSpeed = fabs(currentPanSpeed);
  if (absSpeed < 1.0) return;                 // Stop condition

  int requiredDir = (currentPanSpeed > 0) ? 1 : -1;
  setPanDirection(requiredDir);

  unsigned long nowUs = micros();
  // Calculate microsecond pulse interval dynamically from target step rate
  unsigned long intervalUs = (unsigned long)(1000000.0 / absSpeed);

  if (nowUs - lastStepUs >= intervalUs) {
    lastStepUs = nowUs;
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_PULSE_US);
    digitalWrite(STEP_PIN, LOW);
  }
}

void handleServo() {
  if (servoCurrent == servoTarget) return;
  unsigned long now = millis();
  if (now - lastServoStepMs >= SERVO_STEP_DELAY_MS) {
    lastServoStepMs = now;
    servoCurrent += (servoCurrent < servoTarget) ? 1 : -1;
    tiltServo.write(servoCurrent);
  }
}

void handleUdpCommands() {
  int size = udp.parsePacket();
  while (size > 0) {
    piIP = udp.remoteIP();

    char buf[64];
    int len = udp.read(buf, sizeof(buf) - 1);
    if (len < 0) len = 0;
    buf[len] = '\0';

    int speedInput, tiltInput;
    if (sscanf(buf, "CTRL %d %d", &speedInput, &tiltInput) == 2) {
      currentPanSpeed = (float)speedInput;
      servoTarget = constrain(tiltInput, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
      lastCtrlMs  = millis();
    }
    size = udp.parsePacket();
  }
}

bool wasNear[4] = {false, false, false, false};
unsigned long lastSent[4] = {0, 0, 0, 0};

void checkAndReport(int id, ld2410 &radar) {
  bool presence   = radar.presenceDetected();
  bool moving     = radar.movingTargetDetected();
  bool stationary = radar.stationaryTargetDetected();
  int movDist  = moving ? radar.movingTargetDistance() : -1;
  int statDist = stationary ? radar.stationaryTargetDistance() : -1;

  int bestDist = -1;
  bool isMoving = false;
  if (movDist >= 0) { bestDist = movDist; isMoving = true; }
  if (statDist >= 0 && (bestDist < 0 || statDist < bestDist)) { bestDist = statDist; isMoving = false; }

  bool near = presence && bestDist >= 0 && bestDist <= ALERT_DISTANCE_CM;
  unsigned long now = millis();

  if (near && (!wasNear[id] || now - lastSent[id] >= 500)) {
    sendToPi("RADAR " + String(id) + ": " + String(isMoving ? "MOVING" : "STATIONARY") +
             " object at " + String(bestDist) + "cm");
    lastSent[id] = now;
  } else if (!near && wasNear[id]) {
    sendToPi("RADAR " + String(id) + ": clear");
  }
  wasNear[id] = near;
}

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  setPanDirection(1);

  tiltServo.setPeriodHertz(50);
  tiltServo.attach(SERVO_PIN, 500, 2400);
  tiltServo.write(servoCurrent);

  SerialR1.begin(256000, SERIAL_8N1, R1_RX, R1_TX);
  SerialR2.begin(256000, SERIAL_8N1, R2_RX, R2_TX);
  SerialR3.begin(256000, SERIAL_8N1, R3_RX, R3_TX);
  delay(500);

  radar1.begin(SerialR1, false);
  radar2.begin(SerialR2, false);
  radar3.begin(SerialR3, false);

  connectWiFi();
  udp.begin(CMD_PORT);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    currentPanSpeed = 0.0;
    connectWiFi();
    udp.begin(CMD_PORT);
  }

  radar1.read();
  radar2.read();
  radar3.read();

  handleUdpCommands();

  // Watchdog safety stop
  if (fabs(currentPanSpeed) > 0.0 && (millis() - lastCtrlMs > CTRL_TIMEOUT_MS)) {
    currentPanSpeed = 0.0;
  }

  handlePan();
  handleServo();

  unsigned long now = millis();

  static unsigned long lastRead = 0;
  if (now - lastRead >= READ_INTERVAL_MS) {
    lastRead = now;
    checkAndReport(1, radar1);
    checkAndReport(2, radar2);
    checkAndReport(3, radar3);
  }

  static unsigned long lastHello = 0;
  if (now - lastHello >= 1000) {
    lastHello = now;
    sendToPi("ESP32 HELLO pan=" + String((int)currentPanSpeed) + " tilt=" + String(servoCurrent));
  }
}