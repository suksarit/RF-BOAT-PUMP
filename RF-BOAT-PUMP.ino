/************************************************************
 * Project : 433 MHz Pump Boat Control
 * Board   : Arduino Nano 328P
 * RF RX   : RXB6
 * Remote  : EV1527 / PT2262
 ************************************************************/

#include <RCSwitch.h>
#include <Servo.h>

/* =========================================================
 * PIN CONFIGURATION
 * ========================================================= */
#define PIN_RF_RX 2
#define PIN_RELAY_START 3
#define PIN_RELAY_IGN 4
#define PIN_SERVO 9
#define PIN_BUZZER 8  // Active Buzzer

/* =========================================================
 * OBJECTS
 * ========================================================= */
RCSwitch rf;
Servo throttleServo;

/* =========================================================
 * SERVO CONFIGURATION
 * ========================================================= */
static int throttlePos = 30;

#define SERVO_MIN 30
#define SERVO_MAX 120
#define SERVO_STEP 2

/* =========================================================
 * RF COMMAND CODES (ตัวอย่าง ต้องแก้ไขในอุปกรณ์จริง)
 * ========================================================= */
#define RF_CMD_THROTTLE_UP 111111
#define RF_CMD_THROTTLE_DOWN 222222
#define RF_CMD_STARTER 333333
#define RF_CMD_IGNITION 444444

/* =========================================================
 * TIMING
 * ========================================================= */
static unsigned long lastIgnitionToggleMs = 0;
#define IGNITION_DEBOUNCE_MS 300

static unsigned long lastThrottleStepMs = 0;
#define THROTTLE_HOLD_INTERVAL_MS 120

/* =========================================================
 * RF HOLD SAFETY TIMEOUT
 * ========================================================= */
static unsigned long lastRFReceiveMs = 0;
#define RF_HOLD_TIMEOUT_MS 250

/* =========================================================
 * STARTER CONTROL (SAFE PULSE)
 * ========================================================= */
static bool starterActive = false;
static unsigned long starterStartMs = 0;
#define STARTER_PULSE_MS 1500

/* =========================================================
 * IGNITION STATE (LOGIC)
 * ========================================================= */
static bool ignitionOn = false;

/* =========================================================
 * BUZZER CONTROL (NON-BLOCKING)
 * ========================================================= */
static bool buzzerActive = false;
static bool buzzerState = false;
static unsigned long buzzerLastMs = 0;
static uint8_t buzzerCount = 0;
static uint8_t buzzerTarget = 0;
static unsigned long buzzerIntervalMs = 0;

#define BUZZER_SHORT_MS 120  // บี๊บสั้น
#define BUZZER_LONG_MS 600   // บี๊บยาว

/* =========================================================
 * THROTTLE HOLD STATE
 * ========================================================= */
static bool throttleUpHeld = false;
static bool throttleDownHeld = false;

/* =========================================================
 * FUNCTION PROTOTYPES
 * ========================================================= */
void initHardware(void);
void initRF(void);
void processRF(void);
void processThrottleHold(void);
void updateStarter(void);
void updateBuzzer(void);

void throttleIncrease(void);
void throttleDecrease(void);
void starterOn(void);
void ignitionToggle(void);
void buzzerStart(uint8_t count, unsigned long intervalMs);

/* =========================================================
 * HARDWARE INITIALIZATION
 * ========================================================= */
void initHardware(void) {
  pinMode(PIN_RELAY_START, OUTPUT);
  pinMode(PIN_RELAY_IGN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  digitalWrite(PIN_RELAY_START, LOW);
  digitalWrite(PIN_RELAY_IGN, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  ignitionOn = false;

  throttleServo.attach(PIN_SERVO);
  throttleServo.write(throttlePos);
}

/* =========================================================
 * RF INITIALIZATION
 * ========================================================= */
void initRF(void) {
  rf.enableReceive(digitalPinToInterrupt(PIN_RF_RX));
}

/* =========================================================
 * RF PROCESSING
 * ========================================================= */
void processRF(void) {
  if (!rf.available()) return;

  unsigned long code = rf.getReceivedValue();
  rf.resetAvailable();

  lastRFReceiveMs = millis();

  throttleUpHeld = false;
  throttleDownHeld = false;

  if (code == RF_CMD_THROTTLE_UP) {
    throttleUpHeld = true;
  } else if (code == RF_CMD_THROTTLE_DOWN) {
    throttleDownHeld = true;
  } else if (code == RF_CMD_STARTER) {
    starterOn();
  } else if (code == RF_CMD_IGNITION) {
    ignitionToggle();
  }
}

/* =========================================================
 * THROTTLE HOLD PROCESSING
 * ========================================================= */
void processThrottleHold(void) {
  unsigned long now = millis();

  if (now - lastRFReceiveMs > RF_HOLD_TIMEOUT_MS) {
    throttleUpHeld = false;
    throttleDownHeld = false;
    return;
  }

  if (now - lastThrottleStepMs < THROTTLE_HOLD_INTERVAL_MS) return;

  if (throttleUpHeld) {
    throttleIncrease();
    lastThrottleStepMs = now;
  } else if (throttleDownHeld) {
    throttleDecrease();
    lastThrottleStepMs = now;
  }
}

/* =========================================================
 * STARTER UPDATE
 * ========================================================= */
void updateStarter(void) {
  if (!starterActive) return;

  if (millis() - starterStartMs >= STARTER_PULSE_MS) {
    digitalWrite(PIN_RELAY_START, LOW);
    starterActive = false;
  }
}

/* =========================================================
 * BUZZER UPDATE (NON-BLOCKING)
 * ========================================================= */
void updateBuzzer(void) {
  if (!buzzerActive) return;

  if (millis() - buzzerLastMs >= buzzerIntervalMs) {
    buzzerLastMs = millis();
    buzzerState = !buzzerState;
    digitalWrite(PIN_BUZZER, buzzerState);

    if (!buzzerState) {
      buzzerCount++;
      if (buzzerCount >= buzzerTarget) {
        buzzerActive = false;
        digitalWrite(PIN_BUZZER, LOW);
      }
    }
  }
}

/* =========================================================
 * ACTION HANDLERS
 * ========================================================= */
void throttleIncrease(void) {
  throttlePos += SERVO_STEP;
  throttlePos = constrain(throttlePos, SERVO_MIN, SERVO_MAX);
  throttleServo.write(throttlePos);
}

void throttleDecrease(void) {
  throttlePos -= SERVO_STEP;
  throttlePos = constrain(throttlePos, SERVO_MIN, SERVO_MAX);
  throttleServo.write(throttlePos);
}

void buzzerStart(uint8_t count, unsigned long intervalMs) {
  if (buzzerActive) return;

  buzzerActive = true;
  buzzerState = true;  // เริ่มดังทันที
  buzzerCount = 0;
  buzzerTarget = count;
  buzzerIntervalMs = intervalMs;
  buzzerLastMs = millis();

  digitalWrite(PIN_BUZZER, HIGH);
}

void starterOn(void) {
  // Ignition OFF + กดสตาร์ต → บี๊บสั้น 3 ครั้ง
  if (!ignitionOn) {
    buzzerStart(3, BUZZER_SHORT_MS);
    return;
  }

  if (starterActive) return;

  starterActive = true;
  starterStartMs = millis();
  digitalWrite(PIN_RELAY_START, HIGH);
}

void ignitionToggle(void) {
  unsigned long now = millis();

  if (now - lastIgnitionToggleMs < IGNITION_DEBOUNCE_MS) return;

  lastIgnitionToggleMs = now;

  ignitionOn = !ignitionOn;
  digitalWrite(PIN_RELAY_IGN, ignitionOn ? HIGH : LOW);

  if (ignitionOn) {
    // เปิดกุญแจ → บี๊บยาว 1 ครั้ง
    buzzerStart(1, BUZZER_LONG_MS);
  } else {
    // ปิดกุญแจ → บี๊บสั้น 1 ครั้ง
    buzzerStart(1, BUZZER_SHORT_MS);
  }
}

/* =========================================================
 * ARDUINO ENTRY POINT
 * ========================================================= */
void setup(void) {
  Serial.begin(9600);
  initHardware();
  initRF();

  Serial.println(F("433 MHz Pump Boat Control READY"));
}

void loop(void) {
  processRF();
  processThrottleHold();
  updateStarter();
  updateBuzzer();
}
