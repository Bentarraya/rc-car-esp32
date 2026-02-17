// #include <DabbleESP32.h>

// // Pin motor TB6612
// #define AIN1 14
// #define AIN2 12
// #define BIN1 18
// #define BIN2 19
// #define PWMA 27
// #define PWMB 33
// #define STBY 13

// // HC-SR04
// #define TRIG_PIN 4
// #define ECHO_PIN 2

// // PWM
// const int PWM_FREQ = 20000;
// const int PWM_RES = 8;
// const int PWM_CH_A = 0;
// const int PWM_CH_B = 1;

// // Speeds
// int BASE_SPEED = 220;

// // Trim
// int TRIM_A = -50;      // motor kanan
// int TRIM_B = 0;        // motor kiri

// // Safety
// int SAFE_CM = 15;

// // Sensor refresh
// unsigned long lastSensorMs = 0;
// long lastDistanceCm = 999;
// const int SENSOR_INTERVAL_MS = 25;

// // Speed calibration (ganti nilai ini setelah kalibrasi)
// float kSpeedPerPWM = 0.0013;   // meter/s per PWM unit

// float decel_est = 0.8;         // m/s^2
// int reversePWM = 160;          // burst reverse speed
// int reverseDurationMs = 180;   // waktu reverse burst

// // -------------------- SETUP --------------------
// void setup() {
//   Serial.begin(115200);

//   pinMode(AIN1, OUTPUT);
//   pinMode(AIN2, OUTPUT);
//   pinMode(BIN1, OUTPUT);
//   pinMode(BIN2, OUTPUT);
//   pinMode(STBY, OUTPUT);

//   pinMode(TRIG_PIN, OUTPUT);
//   pinMode(ECHO_PIN, INPUT);

//   ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
//   ledcAttachPin(PWMA, PWM_CH_A);

//   ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
//   ledcAttachPin(PWMB, PWM_CH_B);

//   digitalWrite(STBY, HIGH);

//   Dabble.begin("ESP32_RC");
//   Serial.println("READY: X=MAJU, O=MUNDUR, Left=LEFT, Right=RIGHT");
// }

// // -------------------- FAST SENSOR --------------------
// long getDistanceFast() {
//   unsigned long now = millis();
//   if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
//     lastSensorMs = now;

//     digitalWrite(TRIG_PIN, LOW);
//     delayMicroseconds(2);
//     digitalWrite(TRIG_PIN, HIGH);
//     delayMicroseconds(10);
//     digitalWrite(TRIG_PIN, LOW);

//     long dur = pulseIn(ECHO_PIN, HIGH, 25000);
//     if (dur == 0) lastDistanceCm = 999;
//     else lastDistanceCm = dur / 58;
//   }

//   return lastDistanceCm;
// }

// // ---------------- MOTOR LOGIC ----------------
// void motorA_forward(int speed) {
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, LOW);
//   ledcWrite(PWM_CH_A, constrain(speed + TRIM_A, 0, 255));
// }

// void motorA_backward(int speed) {
//   digitalWrite(AIN1, LOW);
//   digitalWrite(AIN2, HIGH);
//   ledcWrite(PWM_CH_A, constrain(speed + TRIM_A, 0, 255));
// }

// void motorB_forward(int speed) {
//   digitalWrite(BIN1, LOW);
//   digitalWrite(BIN2, HIGH);
//   ledcWrite(PWM_CH_B, constrain(speed + TRIM_B, 0, 255));
// }

// void motorB_backward(int speed) {
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, LOW);
//   ledcWrite(PWM_CH_B, constrain(speed + TRIM_B, 0, 255));
// }

// void stopAll() {
//   ledcWrite(PWM_CH_A, 0);
//   ledcWrite(PWM_CH_B, 0);

//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, HIGH);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, HIGH);
// }

// // ---------------- MOVEMENT ----------------
// void forward_raw() {
//   motorA_forward(BASE_SPEED);
//   motorB_forward(BASE_SPEED);
// }

// void backward() {
//   motorA_backward(BASE_SPEED);
//   motorB_backward(BASE_SPEED);
// }

// void turnLeft() {
//   motorA_forward(BASE_SPEED);
//   motorB_backward(BASE_SPEED);
// }

// void turnRight() {
//   motorA_backward(BASE_SPEED);
//   motorB_forward(BASE_SPEED);
// }

// // --------------- EMERGENCY STOP + TTC ----------------
// float estimateSpeed(int pwm) {
//   return pwm * kSpeedPerPWM;
// }

// void emergencyBrakeAndReverse() {
//   Serial.println("!!! EMERGENCY BRAKE !!!");

//   // Hard brake
//   ledcWrite(PWM_CH_A, 0);
//   ledcWrite(PWM_CH_B, 0);
//   delay(20);

//   // Reverse burst
//   digitalWrite(AIN1, LOW);
//   digitalWrite(AIN2, HIGH);
//   digitalWrite(BIN1, HIGH);
//   digitalWrite(BIN2, LOW);

//   ledcWrite(PWM_CH_A, reversePWM);
//   ledcWrite(PWM_CH_B, reversePWM);
//   delay(reverseDurationMs);

//   // Stop final
//   stopAll();
//   delay(80);
// }

// bool forward_with_safety() {
//   long dcm = getDistanceFast();
//   float d = dcm / 100.0;
//   float v = estimateSpeed(BASE_SPEED);

//   float stopping = (v * v) / (2 * decel_est);
//   float buffer = max(0.20f, v * 0.35f);

//   if (d <= (stopping + buffer)) {
//     emergencyBrakeAndReverse();
//     return false;
//   }

//   forward_raw();
//   return true;
// }

// // -------------------- LOOP --------------------
// void loop() {
//   Dabble.processInput();

//   bool X = GamePad.isCrossPressed();
//   bool O = GamePad.isCirclePressed();
//   bool L = GamePad.isLeftPressed();
//   bool R = GamePad.isRightPressed();

//   if (X) {
//     forward_with_safety();
//   }
//   else if (O) {
//     backward();
//   }
//   else if (L) {
//     turnLeft();
//   }
//   else if (R) {
//     turnRight();
//   }
//   else {
//     stopAll();
//   }

//   delay(5);
// }


/* RC Car - Manual + Autonomous (smart)
   - Uses DabbleESP32 gamepad
   - Pin mapping and motor logic as requested
   - Ultrasonic TTC safety + emergency reverse
   - Mode toggle: Select button switches Manual <-> Autonomous
   - Autonomous: Random-walk + obstacle avoidance + short scanning + wall-follow attempt
*/

#include <DabbleESP32.h>

// Pin motor TB6612
#define AIN1 14
#define AIN2 12
#define BIN1 18
#define BIN2 19
#define PWMA 27
#define PWMB 33
#define STBY 13

// HC-SR04
#define TRIG_PIN 4
#define ECHO_PIN 2

// PWM
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int PWM_CH_A = 0;
const int PWM_CH_B = 1;

// Speeds
int BASE_SPEED = 225;     // cruising speed in autonomous
int MANUAL_SPEED = 255;   // speed for manual input
int BACK_SPEED = 220;
int TURN_SPEED = 200;

// Trim (kalau perlu)
int TRIM_A = -35;     // motor kanan
int TRIM_B = 0;       // motor kiri

// Safety / sensor timing
int SAFE_CM = 20;           // baseline safe distance (cm)
unsigned long lastSensorMs = 0;
long lastDistanceCm = 999;
const int SENSOR_INTERVAL_MS = 25; // sensor polling rate

// TTC calibration (kalibrasi manual)
float kSpeedPerPWM = 0.0013f;   // m/s per PWM unit (kalibrasi sendiri)
float decel_est = 0.8f;         // estimated deceleration (m/s^2) when braking/reverse
int reversePWM = 160;
int reverseDurationMs = 180;

// Mode
bool modeManual = true;
unsigned long lastModeToggle = 0;
const unsigned long MODE_DEBOUNCE = 250;

// Autonomous state machine
enum AutoState { AUTO_FORWARD, AUTO_TURN_LEFT, AUTO_TURN_RIGHT, AUTO_BACKUP, AUTO_PAUSE, AUTO_SCAN };
AutoState astate = AUTO_FORWARD;
unsigned long stateUntil = 0;
int scanStep = 0;

// Forward declarations
long getDistanceFast();
float estimateSpeed(int pwm);
void emergencyBrakeAndReverse();
bool forward_with_safety(int pwm);
void motorA_forward(int speed);
void motorA_backward(int speed);
void motorB_forward(int speed);
void motorB_backward(int speed);
void stopAll();
void forward_raw(int speed);
void backward_raw(int speed);
void turnLeft_raw(int speed);
void turnRight_raw(int speed);
void manualControl();
void autonomousStep();
void handleModeToggle();
void printDebugInfo();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH_A);

  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, PWM_CH_B);

  digitalWrite(STBY, HIGH);

  Dabble.begin("ESP32_RC");

  randomSeed(analogRead(0));
  astate = AUTO_FORWARD;
  stateUntil = millis() + random(800, 2000);

  Serial.println("READY: Select toggles Manual/Auto | X=MAJU | O=MUNDUR | Left/Right steer");
}

// -------------------- SENSOR (throttled) --------------------
long getDistanceFast() {
  unsigned long now = millis();
  if (now - lastSensorMs >= SENSOR_INTERVAL_MS) {
    lastSensorMs = now;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long dur = pulseIn(ECHO_PIN, HIGH, 25000); // 25 ms timeout
    if (dur == 0) lastDistanceCm = 999;
    else lastDistanceCm = dur / 58;
  }
  return lastDistanceCm;
}

// -------------------- MOTOR PRIMITIVES --------------------
// Follow wiring / mapping: RIGHT motor (A): forward -> AIN1 HIGH, AIN2 LOW (CW)
// LEFT motor (B): forward -> BIN1 LOW, BIN2 HIGH (CCW)

void motorA_forward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CH_A, constrain(speed + TRIM_A, 0, 255));
}
void motorA_backward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWM_CH_A, constrain(speed + TRIM_A, 0, 255));
}
void motorB_forward(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWM_CH_B, constrain(speed + TRIM_B, 0, 255));
}
void motorB_backward(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWM_CH_B, constrain(speed + TRIM_B, 0, 255));
}

void stopAll() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  // Put both channels in brake mode
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

// raw movement helpers using speed parameter
void forward_raw(int speed) {
  motorA_forward(speed);
  motorB_forward(speed);
}
void backward_raw(int speed) {
  motorA_backward(speed);
  motorB_backward(speed);
}
void turnLeft_raw(int speed) {
  // left = right motor forward, left motor backward
  motorA_forward(speed);
  motorB_backward(speed);
}
void turnRight_raw(int speed) {
  // right = right motor backward, left motor forward
  motorA_backward(speed);
  motorB_forward(speed);
}

// -------------------- SAFETY / TTC --------------------
float estimateSpeed(int pwm) {
  return pwm * kSpeedPerPWM; // m/s
}

void emergencyBrakeAndReverse() {
  Serial.println("EMERGENCY: BRAKE + REVERSE BURST");
  // immediate brake
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  delay(30);

  // reverse burst (both motors backward)
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); // A backward
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); // B backward
  ledcWrite(PWM_CH_A, reversePWM);
  ledcWrite(PWM_CH_B, reversePWM);
  delay(reverseDurationMs);

  // final stop
  stopAll();
  delay(80);
}

// Returns true if safe and forward started; returns false if emergency triggered
bool forward_with_safety(int pwm) {
  long dcm = getDistanceFast();
  float d = dcm / 100.0f; // m
  float v = estimateSpeed(pwm); // m/s
  float stopping = (v * v) / (2.0f * decel_est); // m
  float buffer = max(0.20f, v * 0.35f); // m buffer proportional to speed
  Serial.print("TTC check: dcm="); Serial.print(dcm);
  Serial.print(" stop(m)="); Serial.print(stopping,4);
  Serial.print(" buf(m)="); Serial.println(buffer,4);

  if (d <= (stopping + buffer)) {
    emergencyBrakeAndReverse();
    return false;
  }

  forward_raw(pwm);
  return true;
}

// -------------------- MANUAL CONTROL --------------------
void manualControl() {
  bool X = GamePad.isCrossPressed();
  bool O = GamePad.isCirclePressed();
  bool L = GamePad.isLeftPressed();
  bool R = GamePad.isRightPressed();

  // Safety: check TTC for straight forward or arcs
  if (X) {
    // for arcs use slightly lower pwm
    if (L && !R) {
      // arc left - slower right-left blend
      int arcPWM = (int)(MANUAL_SPEED * 0.75);
      // Use safety check with arcPWM (conservative)
      if (!forward_with_safety(arcPWM)) return;
      // If safe, apply arc by lowering one side
      motorA_forward(arcPWM);                // right motor
      motorB_forward((int)(arcPWM * 0.5));   // left motor slower for arc
      return;
    } else if (R && !L) {
      int arcPWM = (int)(MANUAL_SPEED * 0.75);
      if (!forward_with_safety(arcPWM)) return;
      motorA_forward((int)(arcPWM * 0.5));
      motorB_forward(arcPWM);
      return;
    } else {
      // straight forward with safety and full manual speed
      forward_with_safety(MANUAL_SPEED);
      return;
    }
  }

  if (O) {
    // backward (no TTC for backward)
    backward_raw(MANUAL_SPEED);
    return;
  }

  if (L && !R) {
    // spin left
    turnLeft_raw(TURN_SPEED);
    return;
  }

  if (R && !L) {
    // spin right
    turnRight_raw(TURN_SPEED);
    return;
  }

  // no input
  stopAll();
}

// -------------------- AUTONOMOUS --------------------
void autonomousStep() {
  unsigned long now = millis();

  // emergency reaction before deciding
  long dcm = getDistanceFast();
  if (dcm <= SAFE_CM) {
    // immediate backup + rotate and resume
    Serial.println("AUTO: immediate obstacle - backup+rotate");
    backward_raw(BACK_SPEED);
    delay(400);
    stopAll();
    delay(80);
    if (random(0, 2) == 0) turnLeft_raw(TURN_SPEED), delay(random(350, 900));
    else turnRight_raw(TURN_SPEED), delay(random(350, 900));
    stopAll();
    stateUntil = now + 250;
    astate = AUTO_FORWARD;
    return;
  }

  // if current state expired, pick next action
  if (now >= stateUntil) {
    int r = random(0, 100);
    if (r < 55) { astate = AUTO_FORWARD; stateUntil = now + random(1200, 3000); }
    else if (r < 75) { astate = AUTO_TURN_LEFT; stateUntil = now + random(600, 1600); }
    else if (r < 95) { astate = AUTO_TURN_RIGHT; stateUntil = now + random(600, 1600); }
    else { astate = AUTO_PAUSE; stateUntil = now + random(300, 900); }
  }

  // execute current state
  switch (astate) {
    case AUTO_FORWARD:
      // Use forward_with_safety for TTC-aware forward
      forward_with_safety(BASE_SPEED);
      break;

    case AUTO_TURN_LEFT:
      // gentle turn left (arc)
      turnLeft_raw((int)(BASE_SPEED * 0.7));
      break;

    case AUTO_TURN_RIGHT:
      turnRight_raw((int)(BASE_SPEED * 0.7));
      break;

    case AUTO_BACKUP:
      backward_raw(BACK_SPEED);
      break;

    case AUTO_PAUSE:
      stopAll();
      break;

    case AUTO_SCAN:
      // simple scan: rotate in place left then right
      if (scanStep % 4 == 0) turnLeft_raw(TURN_SPEED);
      else if (scanStep % 4 == 1) stopAll();
      else if (scanStep % 4 == 2) turnRight_raw(TURN_SPEED);
      else stopAll();
      if (millis() >= stateUntil) { scanStep++; stateUntil = millis() + 350; }
      break;
  }
}

// -------------------- MODE TOGGLE (FINAL FIXED) --------------------
void handleModeToggle() {
  static bool lastSelect = false;
  static unsigned long lastToggleTime = 0;
  unsigned long now = millis();
  
  bool currentSelect = GamePad.isSelectPressed();
  
  // Detect rising edge = tombol baru ditekan
  if (currentSelect && !lastSelect) {
    if (now - lastToggleTime > MODE_DEBOUNCE) {

      // Ganti mode
      modeManual = !modeManual;
      lastToggleTime = now;

      // Stop motor total
      stopAll();

      // Reset autonomous state
      astate = AUTO_FORWARD;
      stateUntil = now + 400;
      scanStep = 0;   // biar mode scan nggak lanjut sisa step
     
      // Clear input sudden spikes
      Dabble.processInput();

      Serial.println("\n========================================");
      Serial.print("MODE CHANGED TO: ");
      Serial.println(modeManual ? "MANUAL" : "AUTONOMOUS");
      Serial.println("========================================");
    }
  }

  lastSelect = currentSelect;
}

// -------------------- DEBUG --------------------
void printDebugInfo() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 1500) return;
  last = now;
  Serial.print("Mode=");
  Serial.print(modeManual ? "MANUAL" : "AUTO");
  Serial.print(" | state=");
  switch (astate) {
    case AUTO_FORWARD: Serial.print("FWD"); break;
    case AUTO_TURN_LEFT: Serial.print("TLEFT"); break;
    case AUTO_TURN_RIGHT: Serial.print("TRIGHT"); break;
    case AUTO_BACKUP: Serial.print("BACK"); break;
    case AUTO_PAUSE: Serial.print("PAUSE"); break;
    case AUTO_SCAN: Serial.print("SCAN"); break;
  }
  Serial.print(" | Dist(cm)="); Serial.print(getDistanceFast());
  Serial.print(" | BASE_PWM="); Serial.print(BASE_SPEED);
  Serial.print(" | trimA="); Serial.print(TRIM_A); Serial.print(" trimB="); Serial.print(TRIM_B);
  Serial.println();
}

// -------------------- MAIN LOOP --------------------
void loop() {
  Dabble.processInput();
  handleModeToggle();

  if (modeManual) manualControl();
  else autonomousStep();

  printDebugInfo();
  delay(10);
}

