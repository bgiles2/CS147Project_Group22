#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include "iot_common.h"


// ---------------- MPU-6050 registers (same helpers as Lab 3) ----------------
constexpr uint8_t MPU_PWR_MGMT_1   = 0x6B;  // write 0x00 to wake the chip
constexpr uint8_t MPU_CONFIG       = 0x1A;  // DLPF_CFG: 0x03 = ~44 Hz low-pass filter
constexpr uint8_t MPU_ACCEL_CONFIG = 0x1C;  // AFS_SEL: 0x08 = +/-4 g (walking can pass 2 g)
constexpr uint8_t MPU_WHO_AM_I     = 0x75;  // 0x68 on a genuine part (clones: 0x70/0x72/0x98)
constexpr uint8_t MPU_ACCEL_XOUT_H = 0x3B;  // 6 bytes: XH XL YH YL ZH ZL
constexpr float   ACCEL_LSB_PER_G  = 8192.0f;  // at +/-4 g

//motion detection stuff
constexpr uint32_t SAMPLE_MS      = 20;        // 50 Hz
constexpr float    VIB_ALPHA      = 0.05f;     // smoothing for vibration level
constexpr float    MOVE_HIGH_G    = 0.020f;    // above this = moving
constexpr float    MOVE_LOW_G     = 0.010f;    // below this = still 
constexpr uint32_t START_MS       = 5000;     // motion this long = cycle started
constexpr uint32_t DONE_QUIET_MS  = 5000;    // quiet this long = cycle finished 
constexpr uint32_t ALERT_MS = 10000; //how long DONE should stay on

uint32_t doneMs = 0;

struct AccelG { float x, y, z; };

enum State { IDLE, RUNNING, DONE };
 
//state information
bool     mpuOk = false;
State    state = IDLE;
float    restMag = 1.0f;       // |a| at rest, about 1 g
float    vibrationG = 0.0f;      // smoothed vibration level, in g
bool     moving = false;
uint32_t motionStartMs = 0;    // when the current motion burst began
uint32_t lastMotionMs = 0;     // last time motion was seen
 
//wifi notifcation stuff
//we're gonna need to figure out how to change this depending on the user --> this is just what I was using to test at my place
const char* ssid = "Velocity Wi-Fi"; 
const char* password = "catyoicv";
const char* ntfyTopic = "group22-washing-machine-monitor";

//connecting to wifi + sending notifs
void connectWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);

    Serial.println("\nWiFi connected!");
}

void sendNotification(const char* message) {
    if (WiFi.status() != WL_CONNECTED){
        Serial.println("WiFi disconnected. Notification not sent.");
        return;
    }

    WiFiClientSecure client;
    client.setInsecure();

    HTTPClient http;

    String url = "https://ntfy.sh/" + String(ntfyTopic);

    http.setTimeout(5000);

    if (http.begin(client, url)){
      http.addHeader("Content-Type", "text/plain");
      http.addHeader("Title", "Washing Machine Monitor");

      int responseCode = http.POST(String(message));

      Serial.print("HTTP Response: ");
      Serial.println(responseCode);

      if (responseCode >= 200 && responseCode < 300){
        Serial.println("Notification sent successfully!");
      }
      else{
        Serial.println("Notification failed!");
        Serial.println(http.getString());
      }

      http.end();
    }
    else{
      Serial.println("Unable to connect to ntfy.");
    }

    client.stop();
}

//mpu6050 stuff (this was in the step counter lab already)
bool mpuWriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IoTPins::MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadRegs(uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(IoTPins::MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(IoTPins::MPU6050_ADDR, len) < len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool readAccelG(AccelG& a) {
  uint8_t raw[6];
  if (!mpuReadRegs(MPU_ACCEL_XOUT_H, raw, 6)) return false;
  a.x = (int16_t)((raw[0] << 8) | raw[1]) / ACCEL_LSB_PER_G;
  a.y = (int16_t)((raw[2] << 8) | raw[3]) / ACCEL_LSB_PER_G;
  a.z = (int16_t)((raw[4] << 8) | raw[5]) / ACCEL_LSB_PER_G;
  return true;
}

bool mpuBegin() {
  uint8_t who = 0;
  bool ok = mpuWriteReg(MPU_PWR_MGMT_1, 0x00) &&
            mpuWriteReg(MPU_CONFIG, 0x03) &&
            mpuWriteReg(MPU_ACCEL_CONFIG, 0x08) &&
            mpuReadRegs(MPU_WHO_AM_I, &who, 1);
  Serial.print("WHO_AM_I = 0x");
  Serial.println(who, HEX);
  if (!ok) Serial.println("MPU-6050 did not answer at 0x68 (check AD0->GND, SDA 21 / SCL 22)");
  return ok;
}

//calibrate initial accel value
void calibrate() {
  Serial.println("Calibrating device...");
  float sum = 0;
  int n = 0;
  while (n < 100){
    AccelG a;
    if (readAccelG(a)){
      sum += sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
      n++;
    }
    delay(20);
  }
  restMag = sum / n;
  Serial.printf("Rest magnitude = %.3f g\n", restMag);
}

//update state based on acceleration
void updateMotion(const AccelG& a){
  float mag = sqrtf(a.x * a.x + a.y * a.y + a.z * a.z);
  float dev = fabsf(mag - restMag);
  vibrationG = VIB_ALPHA * dev + (1 - VIB_ALPHA) * vibrationG;
 
  bool wasMoving = moving;
  if (!moving && vibrationG > MOVE_HIGH_G){
    moving = true;
  }
  if (moving && vibrationG < MOVE_LOW_G){
    moving = false;
  }
 
  uint32_t now = millis();
  if (moving){
    if (!wasMoving){
      motionStartMs = now;
    } 
    lastMotionMs = now;
  }
 
  switch (state){
    case IDLE:
      if (moving && now - motionStartMs > START_MS){
        state = RUNNING;
        Serial.println("Cycle STARTED");
        sendNotification("Washing machine has started!");
      }
      break;
    case RUNNING:
      if (!moving && now - lastMotionMs > DONE_QUIET_MS){
        state = DONE;
        doneMs = now;
        Serial.println("Cycle DONE -> sending notifcation!");
        sendNotification("Washing machine cycle is complete!");
      }
      break;
    case DONE:
      if (moving || now - doneMs > ALERT_MS){
        state = IDLE;
        Serial.println("Cycle alert DONE -> back to idle!");
      }
      break;
  }
}
 


void setup() {
  iotStartSerial("Washing machine sensor");
  connectWiFi();
  iotInitI2C();

  pinMode(IoTPins::LED_1, OUTPUT);
  digitalWrite(IoTPins::LED_1, LOW);
 
  mpuOk = mpuBegin();
  if (mpuOk){
    calibrate();
  }
}
 
void loop() {
  static uint32_t lastSample = 0, lastPrint = 0;
 
  if (mpuOk && iotEvery(SAMPLE_MS, lastSample)){
    AccelG a;
    if (readAccelG(a)) updateMotion(a);
    digitalWrite(IoTPins::LED_1, state == DONE ? HIGH : LOW);
  }
  
  //printing vibration/moving/state every 2 seconds
  if (iotEvery(2000, lastPrint)) {
    Serial.printf("vibrationG=%.4f  moving=%d  state=%d\n", vibrationG, moving, state);
  }
}