// E.V.A. - 6-servo robotic arm firmware (Arduino Uno + PCA9685)
// Version 1.0
//
// Design rule: ONE MOTOR AT A TIME.
// Every motion command (ANGLE, ALL, XYZ, HOME) only sets targets. A scheduler then moves a single joint,
// waits a short gap, then moves the next one. Idle joints simply hold their position. This keeps the peak
// current of the 5 V supply low.
//
// Library: Adafruit PWM Servo Driver Library
//
// Serial protocol (115200 baud, line ending = Newline):
//   ANGLE <j> <jointDeg>              move one joint to a real joint angle
//   ALL a0 a1 a2 a3 a4 a5             set all six joint targets (they run one after another)
//   XYZ x y z toolPitch roll grip     inverse kinematics for the wrist pivot (cm, degrees)
//   HOME                              go to the home pose
//   STOP                              stop moving now (holds position)
//   SPEED <1-100>                     speed scale (capped at SPEED_CAP)
//   SERVO <j> <servoDeg>              calibration: move to a raw servo angle
//   RAW <ch> <us>                     calibration: raw pulse (only when idle)
//   RELAX <j>                         stop PWM on a joint (it goes limp)
//   P                                 print 6 joint angles + the index of the moving joint (-1 = none)
//   Q                                 print BUSY or IDLE
//
// Joints: 0 base, 1 shoulder, 2 elbow, 3 roll, 4 pitch, 5 gripper
// Angle conventions:
//   J0 base:     0 = facing +X, positive = toward +Y (left)
//   J1 shoulder: upper arm angle from horizontal, 90 = straight up
//   J2 elbow:    bend vs upper arm, 0 = straight, positive = forearm forward/down
//   J3 roll:     0 = flat/home roll
//   J4 pitch:    bend vs forearm, 0 = straight, positive = forward/down
//   J5 gripper:  0 = half open, positive = closing
//
// The joint table below holds the calibration of MY arm. Re-calibrate yours: docs/03-calibration-guide.md

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ---- Arm dimensions (cm) ----
const float H0 = 9.7;    // base to shoulder pivot
const float L1 = 12.0;   // upper arm
const float L2 = 9.3;    // forearm (elbow to wrist pivot)

// ---- Safety settings ----
const float SERVO_MIN_DEG = 8.0;     // never drive a servo into its end stops
const float SERVO_MAX_DEG = 172.0;
const float SPEED_CAP     = 0.80;    // speed scale can never exceed this
const unsigned long GAP_MS = 150;    // pause between one joint finishing and the next starting
// Order in which joints take their turn when several have a new target
const int ORDER[6] = {1, 2, 0, 4, 3, 5};

struct Joint {
  uint8_t ch;
  int minUs, maxUs;    // pulse at servo 0 / 180 deg
  int8_t dir;          // +1 / -1
  float servoHome;     // servo angle at home pose
  float jointHome;     // joint angle at home pose
  float minJ, maxJ;    // limits (joint angle)
  float maxSpeed;      // deg/s
  float accel;         // deg/s^2
  float posDiv;        // divides motion on the positive side of home (1.0 = no correction)
};

Joint J[6] = {
  // ch minUs maxUs dir servoHome jointHome minJ  maxJ  speed accel posDiv
  {0,  490, 2415,  1,   90,   0,   -80,   80,  30,  60, 1.00},   // base      MG996R
  {1,  600, 2400,  1,   95,  90,    20,  120,  20,  40, 1.18},   // shoulder  MG996R (backward side overshoots ~18%)
  {2,  600, 2400,  1,   55,   0,   -40,  110,  25,  50, 1.00},   // elbow     MG996R
  {3,  500, 2400,  1,  100,   0,   -80,   70,  40, 100, 1.00},   // roll      MG90S  (maxJ 70 keeps it off the end stop)
  {4,  500, 2400, -1,   90,   0,   -80,   80,  40, 100, 1.00},   // pitch     MG90S
  {5,  500, 2400, -1,   90,   0,   -40,   40,  60, 200, 1.00}    // gripper   MG90S
};

float cur[6], tgt[6], vel[6];
bool active[6];
int movingJ = -1;                 // the ONLY joint allowed to move right now (-1 = none)
unsigned long gapUntil = 0;
float speedScale = 0.5;
const float DT = 0.02;            // 20 ms update
unsigned long lastTick = 0;

uint16_t usToTicks(float us) { return (uint16_t)(us * 4096.0 / 20000.0 + 0.5); }

float jointToServo(int i, float a) {
  float d = a - J[i].jointHome;
  if (d > 0) d /= J[i].posDiv;
  float s = J[i].servoHome + J[i].dir * d;
  return constrain(s, SERVO_MIN_DEG, SERVO_MAX_DEG);
}

float servoToJoint(int i, float s) {
  float d = (s - J[i].servoHome) / J[i].dir;
  if (d > 0) d *= J[i].posDiv;
  return J[i].jointHome + d;
}

void writeJoint(int i, float a) {
  float s = jointToServo(i, a);
  float us = J[i].minUs + (J[i].maxUs - J[i].minUs) * (s / 180.0);
  pwm.setPWM(J[i].ch, 0, usToTicks(us));
  active[i] = true;
}

bool inLimits(int i, float a) { return a >= J[i].minJ && a <= J[i].maxJ; }

void setTarget(int i, float a) {
  tgt[i] = constrain(a, J[i].minJ, J[i].maxJ);
}

bool isBusy() {
  if (movingJ >= 0) return true;
  for (int i = 0; i < 6; i++)
    if (active[i] && fabs(tgt[i] - cur[i]) > 0.15) return true;
  return false;
}

void stopAll() {
  for (int i = 0; i < 6; i++) {
    if (i == movingJ) {
      // let the moving joint decelerate to a stop instead of freezing instantly
      float stopDist = vel[i] * fabs(vel[i]) / (2.0 * J[i].accel);
      tgt[i] = constrain(cur[i] + stopDist, J[i].minJ, J[i].maxJ);
    } else {
      tgt[i] = cur[i];
    }
  }
}

// ---- Motion: only ONE joint moves at a time ----
void updateMotion() {
  unsigned long now = millis();

  if (movingJ < 0) {
    if (now < gapUntil) return;
    for (int k = 0; k < 6; k++) {
      int i = ORDER[k];
      if (active[i] && fabs(tgt[i] - cur[i]) > 0.15) { movingJ = i; break; }
    }
    if (movingJ < 0) return;       // nothing to do
  }

  int i = movingJ;
  float err = tgt[i] - cur[i];
  float vmax = J[i].maxSpeed * speedScale;
  float amax = J[i].accel;
  float vWant = sqrt(2.0 * amax * fabs(err));   // fastest speed that can still stop in time
  if (vWant > vmax) vWant = vmax;
  if (err < 0) vWant = -vWant;
  float dv = constrain(vWant - vel[i], -amax * DT, amax * DT);
  vel[i] += dv;

  if (fabs(err) < 0.15 && fabs(vel[i]) < 2.0) {
    cur[i] = tgt[i];
    vel[i] = 0;
    writeJoint(i, cur[i]);
    movingJ = -1;                  // release the turn
    gapUntil = now + GAP_MS;
    return;
  }
  cur[i] += vel[i] * DT;
  writeJoint(i, cur[i]);
}

// ---- Inverse kinematics ----
// toolPitch: gripper direction from horizontal (0 = forward level, -90 = pointing down)
bool moveXYZ(float x, float y, float z, float toolPitch, float roll, float grip) {
  if (fabs(x) < 0.1 && fabs(y) < 0.1) x = 0.1;

  float base = atan2(y, x) * RAD_TO_DEG;
  float r = sqrt(x * x + y * y);
  float zp = z - H0;
  float D = sqrt(r * r + zp * zp);

  if (D > (L1 + L2) - 0.1 || D < fabs(L1 - L2) + 0.1) {
    Serial.println("ERR out of reach");
    return false;
  }

  float alpha = acos(constrain((L1 * L1 + D * D - L2 * L2) / (2 * L1 * D), -1.0, 1.0));
  float beta = atan2(zp, r);
  float shoulder = (beta + alpha) * RAD_TO_DEG;                       // elbow-up
  float gamma = acos(constrain((L1 * L1 + L2 * L2 - D * D) / (2 * L1 * L2), -1.0, 1.0));
  float elbow = (PI - gamma) * RAD_TO_DEG;

  float forearmAngle = shoulder - elbow;       // forearm direction from horizontal
  float wrist = forearmAngle - toolPitch;      // pitch bend needed

  // Order matches joints: base, shoulder, elbow, roll, pitch, gripper
  float a[6] = {base, shoulder, elbow, roll, wrist, grip};
  for (int i = 0; i < 6; i++) {
    if (!inLimits(i, a[i])) {
      Serial.print("ERR joint "); Serial.print(i);
      Serial.print(" needs "); Serial.println(a[i], 1);
      return false;                            // reject instead of moving somewhere wrong
    }
  }
  for (int i = 0; i < 6; i++) setTarget(i, a[i]);
  return true;
}

void handleLine(char *line) {
  char *cmd = strtok(line, " ");
  if (!cmd) return;

  if (!strcasecmp(cmd, "ANGLE")) {
    char *a = strtok(NULL, " "), *b = strtok(NULL, " ");
    if (a && b) setTarget(constrain(atoi(a), 0, 5), atof(b));
  } else if (!strcasecmp(cmd, "ALL")) {
    float v[6];
    for (int i = 0; i < 6; i++) {
      char *t = strtok(NULL, " ");
      if (!t) return;                          // incomplete command: ignore completely
      v[i] = atof(t);
    }
    for (int i = 0; i < 6; i++) setTarget(i, v[i]);
  } else if (!strcasecmp(cmd, "XYZ")) {
    float v[6];
    for (int k = 0; k < 6; k++) {
      char *t = strtok(NULL, " ");
      if (!t) { Serial.println("ERR need 6 values"); return; }
      v[k] = atof(t);
    }
    if (moveXYZ(v[0], v[1], v[2], v[3], v[4], v[5])) Serial.println("OK");
  } else if (!strcasecmp(cmd, "HOME")) {
    for (int i = 0; i < 6; i++) setTarget(i, J[i].jointHome);
  } else if (!strcasecmp(cmd, "STOP")) {
    stopAll();
  } else if (!strcasecmp(cmd, "SPEED")) {
    char *a = strtok(NULL, " ");
    if (a) {
      float s = constrain(atof(a), 1.0, 100.0) / 100.0;
      if (s > SPEED_CAP) s = SPEED_CAP;
      speedScale = s;
    }
  } else if (!strcasecmp(cmd, "SERVO")) {
    char *a = strtok(NULL, " "), *b = strtok(NULL, " ");
    if (a && b) {
      int i = constrain(atoi(a), 0, 5);
      setTarget(i, servoToJoint(i, constrain(atof(b), SERVO_MIN_DEG, SERVO_MAX_DEG)));
    }
  } else if (!strcasecmp(cmd, "RAW")) {
    char *a = strtok(NULL, " "), *b = strtok(NULL, " ");
    if (a && b && !isBusy()) {                 // only when nothing else is moving
      int ch = constrain(atoi(a), 0, 5);
      pwm.setPWM(ch, 0, usToTicks(constrain(atoi(b), 500, 2500)));
    }
  } else if (!strcasecmp(cmd, "RELAX")) {
    char *a = strtok(NULL, " ");
    if (a) { int i = constrain(atoi(a), 0, 5); pwm.setPWM(J[i].ch, 0, 4096); active[i] = false; }
  } else if (!strcasecmp(cmd, "Q")) {
    Serial.println(isBusy() ? "BUSY" : "IDLE");
  } else if (!strcasecmp(cmd, "P")) {
    for (int i = 0; i < 6; i++) { Serial.print(cur[i], 1); Serial.print(' '); }
    Serial.println(movingJ);
  }
}

void setup() {
  Serial.begin(115200);
  pwm.begin();
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);             // don't let an I2C glitch hang the board
#endif
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  delay(10);

  // Start with the arm resting at home. Wake joints one at a time to limit inrush current.
  for (int i = 0; i < 6; i++) {
    cur[i] = tgt[i] = J[i].jointHome;
    vel[i] = 0;
    writeJoint(i, cur[i]);
    delay(400);
  }
  Serial.println("Ready");
}

void loop() {
  static char buf[80];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) buf[len++] = c;
    else len = 0;                              // overlong line: discard
  }

  unsigned long now = millis();
  if (now - lastTick >= 20) {
    lastTick = now;
    updateMotion();
  }
}
