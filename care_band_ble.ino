#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---- BLE (added) ----
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic* pChar = nullptr;
bool deviceConnected = false;

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer* s) {
    deviceConnected = true;
    Serial.println("[BLE] Phone connected");
  }
  void onDisconnect(BLEServer* s) {
    deviceConnected = false;
    Serial.println("[BLE] Phone disconnected, advertising again");
    BLEDevice::startAdvertising();
  }
};

void setupBLE() {
  BLEDevice::init("CARE-BAND");
  BLEServer* server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService* service = server->createService(SERVICE_UUID);
  pChar = service->createCharacteristic(CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pChar->addDescriptor(new BLE2902());
  service->start();
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising as CARE-BAND");
}

// Sends WATER / FOOD / RESTROOM / EMERGENCY to the phone
void sendEvent(const char* msg) {
  Serial.print("[BLE] Sending: ");
  Serial.println(msg);
  if (deviceConnected && pChar) {
    pChar->setValue((uint8_t*)msg, strlen(msg));
    pChar->notify();
  }
}

// ==========================================
// 1. HARDWARE PIN DEFINITIONS (ESP32-C3)
// ==========================================
const int FLEX_PIN = 0;   // GPIO0 (ADC) - Flex sensor voltage divider
const int TOUCH_PIN = 4;  // GPIO4 (Digital/Capacitive sig) - Confirmation touch sensor
const int MOTOR_PIN = 5;  // GPIO5 (PWM/Digital) - ERM Vibration Motor
const int I2C_SDA = 8;    // GPIO8 - MPU6050 SDA
const int I2C_SCL = 9;    // GPIO9 - MPU6050 SCL

// ==========================================
// 2. THRESHOLDS & TIMING CONFIGURATION
// ==========================================
const int FLEX_REST_VALUE = 4095;                   // Unflexed baseline
const int FLEX_DROP_THRESHOLD = 500;                // Drop required from 4095
const unsigned long CONFIRMATION_WINDOW_MS = 5000;  // 5 seconds to confirm

// IMU Emergency Thresholds
const float EMERGENCY_GYRO_THRESHOLD = 50.0;
const float EMERGENCY_ACCEL_DELTA = 3.0;

// ==========================================
// 3. STATE MACHINE DEFINITIONS
// ==========================================
enum SystemState {
  STATE_IDLE,
  STATE_WAITING_CONFIRMATION,
  STATE_EMERGENCY
};

enum RequestType {
  REQ_WATER,
  REQ_FOOD,
  REQ_RESTROOM
};

SystemState currentState = STATE_IDLE;
RequestType activeRequest = REQ_WATER;

unsigned long confirmationStartTime = 0;
bool lastTouchState = LOW;

// Sequence tracking variables for Water, Food, and Restroom
int flexPeakCount = 0;
unsigned long lastFlexTime = 0;
bool wasFlexed = false;

// MPU6050 object
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[INIT] Starting CARE-BAND Firmware (Sequence-Based Multi-Gesture)...");

  pinMode(FLEX_PIN, INPUT);
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 Not Found! Check SDA:8, SCL:9 wiring.");
  } else {
    Serial.println("[INIT] MPU6050 initialized successfully.");
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  setupBLE();  // added

  triggerHapticFeedback(2);
  Serial.println("[READY] System operational.\n");
}

void loop() {
  int rawFlex = analogRead(FLEX_PIN);
  int touchState = digitalRead(TOUCH_PIN);

  sensors_event_t a, g, temp;
  bool imuActive = mpu.getEvent(&a, &g, &temp);

  bool touchPressed = (touchState == HIGH && lastTouchState == LOW);
  lastTouchState = touchState;

  int flexDrop = FLEX_REST_VALUE - rawFlex;
  bool isFlexedNow = (flexDrop > FLEX_DROP_THRESHOLD);

  float totalAccel = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
  float accelDelta = abs(totalAccel - 9.8);

  // Print telemetry
  Serial.print("[TELEMETRY] FlexDrop: ");
  Serial.print(flexDrop);
  Serial.print(" | Touch: ");
  Serial.print(touchState);
  Serial.print(" | FlexCount: ");
  Serial.print(flexPeakCount);
  Serial.println();

  switch (currentState) {

    case STATE_IDLE:
      // 1. Emergency Detection (IMU Jerk/Shake)
      if (imuActive && (abs(g.gyro.x) > EMERGENCY_GYRO_THRESHOLD || abs(g.gyro.y) > EMERGENCY_GYRO_THRESHOLD || accelDelta > EMERGENCY_ACCEL_DELTA)) {
        Serial.println("[ALERT] Emergency gesture detected via IMU!");
        currentState = STATE_EMERGENCY;
        break;
      }

      // 2. Count Flex Bends (Recognizes rising edge of each flex)
      if (isFlexedNow && !wasFlexed) {
        unsigned long now = millis();
        if (now - lastFlexTime < 1400) {
          flexPeakCount++;
        } else {
          flexPeakCount = 1;
        }
        lastFlexTime = now;
      }
      wasFlexed = isFlexedNow;

      // 3. Evaluate Sequence after user finishes bending and pauses for 1.1 seconds
      if (flexPeakCount > 0 && (millis() - lastFlexTime > 1100)) {
        if (flexPeakCount >= 3) {
          Serial.println("[TRIGGER] Triple Flex -> I have to use the restroom");
          activeRequest = REQ_RESTROOM;
          triggerHapticFeedback(3);  // 3 pulses prompt
        } else if (flexPeakCount == 2) {
          Serial.println("[TRIGGER] Double Flex -> I want to eat");
          activeRequest = REQ_FOOD;
          triggerHapticFeedback(2);  // 2 pulses prompt
        } else if (flexPeakCount == 1) {
          Serial.println("[TRIGGER] Single Flex -> I need water");
          activeRequest = REQ_WATER;
          triggerHapticFeedback(1);  // 1 pulse prompt
        }

        confirmationStartTime = millis();
        currentState = STATE_WAITING_CONFIRMATION;
        flexPeakCount = 0;  // Reset count
      }
      break;

    case STATE_WAITING_CONFIRMATION:
      if (imuActive && (abs(g.gyro.x) > EMERGENCY_GYRO_THRESHOLD || abs(g.gyro.y) > EMERGENCY_GYRO_THRESHOLD || accelDelta > EMERGENCY_ACCEL_DELTA)) {
        Serial.println("[ALERT] Emergency override during confirmation!");
        currentState = STATE_EMERGENCY;
        flexPeakCount = 0;
        break;
      }

      // Patient confirms with a separate tap on the touch sensor
      if (touchPressed) {
        Serial.print("[CONFIRMATION SUCCESS] Request validated: ");
        if (activeRequest == REQ_WATER) {
          Serial.println("-> REQUEST: I NEED WATER");
          sendEvent("WATER");        // added
          triggerHapticFeedback(2);  // 2 pulses on success
        } else if (activeRequest == REQ_FOOD) {
          Serial.println("-> REQUEST: I WANT TO EAT");
          sendEvent("FOOD");         // added
          triggerHapticFeedback(3);  // 3 pulses on success
        } else if (activeRequest == REQ_RESTROOM) {
          Serial.println("-> REQUEST: I HAVE TO USE THE RESTROOM");
          sendEvent("RESTROOM");     // added
          triggerHapticFeedback(4);  // 4 pulses on success
        }

        currentState = STATE_IDLE;
        flexPeakCount = 0;
        delay(1000);
      } else if (millis() - confirmationStartTime > CONFIRMATION_WINDOW_MS) {
        Serial.println("[TIMEOUT] Confirmation window expired. Resetting silently.");
        currentState = STATE_IDLE;
        flexPeakCount = 0;
      }
      break;

    case STATE_EMERGENCY:
      Serial.println("[DISPATCH] HIGH-PRIORITY EMERGENCY ALERT TRIGGERED!");
      Serial.println("-> EMERGENCY: IMMEDIATE ASSISTANCE REQUIRED");
      sendEvent("EMERGENCY");  // added
      triggerHapticFeedback(4);
      currentState = STATE_IDLE;
      flexPeakCount = 0;
      delay(2000);
      break;
  }

  delay(50);
}

// ==========================================
// 4. HELPER FUNCTIONS
// ==========================================
void triggerHapticFeedback(int pulseCount) {
  for (int i = 0; i < pulseCount; i++) {
    digitalWrite(MOTOR_PIN, HIGH);
    delay(250);
    digitalWrite(MOTOR_PIN, LOW);
    if (i < pulseCount - 1) {
      delay(250);  // Pause between pulses
    }
  }
}
