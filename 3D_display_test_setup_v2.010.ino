// 3D_display_test_setup_v2.010 (Teensy 4.1 - custom run-break timings)

// ================== Pin Definitions ==================
const int RELAY1       = 23;
const int LED1_REV     = 8;
const int IR1_PIN      = 3;
const int MOSFET1_PIN  = 21;

const int RELAY2       = 22;
const int LED2_REV     = 7;
const int IR2_PIN      = 4;
const int MOSFET2_PIN  = 20;

const int Servo_IN     = 14;

// ================== Timings ==================
const unsigned long STOP_MS         = 1000;   // wait when white detected
const unsigned long RELAY_SETTLE_MS = 80;

// --- Custom pulse times ---
const unsigned long MOTOR1_ON_MS    = 100;
const unsigned long MOTOR1_OFF_MS   = 9900;

const unsigned long MOTOR2_ON_MS    = 100;
const unsigned long MOTOR2_OFF_MS   = 900;

const unsigned long SERVO_STEP_MS   = 100;    // ON/OFF toggle
const unsigned long SERVO_BREAK_MS  = 800;   // pause after 1 full cycle

// ================== Motor State Machine ==================
enum MotorState {
  M_IDLE,
  M_RUN,
  M_STOP_ON_WHITE,
  M_RELAY_TOGGLE,
  M_WAIT_BLACK
};

struct Motor {
  int relayPin;
  int ledRevPin;
  int irPin;
  int mosfetPin;

  MotorState state;
  unsigned long stateStart;
  bool relayState;   // current relay direction
  bool lastIr;       // last IR state

  // --- pulsing ---
  bool pulseOn;
  unsigned long pulseStart;
  unsigned long onTime;
  unsigned long offTime;
};

// ================== Helpers ==================
inline bool readIR(int pin) {
  return digitalRead(pin); // HIGH = white, LOW = black
}

void initMotor(Motor &m, unsigned long onMs, unsigned long offMs) {
  pinMode(m.relayPin, OUTPUT);
  pinMode(m.ledRevPin, OUTPUT);
  pinMode(m.irPin, INPUT);
  pinMode(m.mosfetPin, OUTPUT);

  digitalWrite(m.relayPin, LOW);
  digitalWrite(m.ledRevPin, LOW);
  digitalWrite(m.mosfetPin, LOW);

  m.state = M_IDLE;
  m.stateStart = millis();
  m.relayState = false;
  m.lastIr = readIR(m.irPin);

  m.pulseOn = true;
  m.pulseStart = millis();
  m.onTime = onMs;
  m.offTime = offMs;
}

void motorOn(Motor &m)  { digitalWrite(m.mosfetPin, HIGH); }
void motorOff(Motor &m) { digitalWrite(m.mosfetPin, LOW);  }

void setRelay(Motor &m, bool on) {
  digitalWrite(m.relayPin, on ? HIGH : LOW);
  digitalWrite(m.ledRevPin, on ? HIGH : LOW);
  m.relayState = on;
}

// ================== Motor Logic ==================
void tickMotor(Motor &m, const char *tag) {
  bool ir = readIR(m.irPin); // HIGH = white, LOW = black
  unsigned long now = millis();

  switch (m.state) {
    case M_IDLE:
      Serial.print(tag); Serial.println(": Start run forward");
      setRelay(m, false);
      motorOn(m);
      m.state = M_RUN;
      m.pulseOn = true;
      m.pulseStart = now;
      break;

    case M_RUN:
      // --- Custom run-break pulsing ---
      if (m.pulseOn && (now - m.pulseStart >= m.onTime)) {
        motorOff(m);
        m.pulseOn = false;
        m.pulseStart = now;
        Serial.print(tag); Serial.println(": Pulse OFF");
      } 
      else if (!m.pulseOn && (now - m.pulseStart >= m.offTime)) {
        motorOn(m);
        m.pulseOn = true;
        m.pulseStart = now;
        Serial.print(tag); Serial.println(": Pulse ON");
      }

      if (ir == HIGH && m.lastIr == LOW) { // white just appeared
        Serial.print(tag); Serial.println(": White detected -> stop");
        motorOff(m);
        m.state = M_STOP_ON_WHITE;
        m.stateStart = now;
      }
      break;

    case M_STOP_ON_WHITE:
      if (now - m.stateStart >= STOP_MS) {
        Serial.print(tag); Serial.println(": Toggle relay, resume run");
        setRelay(m, !m.relayState);
        m.state = M_RELAY_TOGGLE;
        m.stateStart = now;
      }
      break;

    case M_RELAY_TOGGLE:
      if (now - m.stateStart >= RELAY_SETTLE_MS) {
        motorOn(m);
        m.state = M_WAIT_BLACK;
        m.pulseOn = true;
        m.pulseStart = now;
      }
      break;

    case M_WAIT_BLACK:
      if (ir == LOW) { // black detected again -> safe
        Serial.print(tag); Serial.println(": Black detected -> back to RUN");
        m.state = M_RUN;
        m.pulseOn = true;
        m.pulseStart = now;
      }
      break;
  }

  m.lastIr = ir;
}

// ================== Servo ==================
#include <PWMServo.h>
PWMServo myServo;

const int SERVO_MIN_POS = 0;
const int SERVO_MAX_POS = 20;

unsigned long lastServoToggle = 0;
bool servoAtMax = false;
bool servoPaused = false;
unsigned long servoPauseStart = 0;

void tickServo() {
  unsigned long now = millis();

  if (servoPaused) {
    if (now - servoPauseStart >= SERVO_BREAK_MS) {
      servoPaused = false;
      lastServoToggle = now;
    }
    return;
  }

  if (now - lastServoToggle >= SERVO_STEP_MS) {
    lastServoToggle = now;
    if (servoAtMax) {
      myServo.write(SERVO_MIN_POS);
      servoAtMax = false;
      Serial.println("Servo -> 0°");
      // completed 1 full cycle (500ms + 500ms)
      servoPaused = true;
      servoPauseStart = now;
    } else {
      myServo.write(SERVO_MAX_POS);
      servoAtMax = true;
      Serial.println("Servo -> 20°");
    }
  }
}

// ================== Globals ==================
Motor motor1 = {RELAY1, LED1_REV, IR1_PIN, MOSFET1_PIN};
Motor motor2 = {RELAY2, LED2_REV, IR2_PIN, MOSFET2_PIN};

// ================== Setup & Loop ==================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("System started");

  initMotor(motor1, MOTOR1_ON_MS, MOTOR1_OFF_MS);
  initMotor(motor2, MOTOR2_ON_MS, MOTOR2_OFF_MS);

  myServo.attach(Servo_IN);
  myServo.write(SERVO_MIN_POS);
  lastServoToggle = millis();
}

void loop() {
  tickMotor(motor1, "Motor1");
  tickMotor(motor2, "Motor2");
  tickServo();
  delay(5);
}
