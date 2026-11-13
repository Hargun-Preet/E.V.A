// Firmware simulation tests. Build and run with tests/firmware_sim/run.sh
#include "Arduino.h"
unsigned long fake_ms = 0;
SerialT Serial;
uint16_t pwm_ticks[16];
#include "../../firmware/EVA_firmware/EVA_firmware.ino"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } else printf("  ok:   %s\n", msg); } while (0)

static void send(const char *cmd) { Serial.feed(std::string(cmd) + "\n"); for (int i = 0; i < 5; i++) { fake_ms++; loop(); } }

// Runs the firmware for `ms` milliseconds and returns the largest number of joints that moved in one 20 ms tick.
static int run(unsigned ms) {
  float prev[6]; for (int i = 0; i < 6; i++) prev[i] = cur[i];
  int worst = 0;
  for (unsigned t = 0; t < ms; t++) {
    fake_ms++; loop();
    if (fake_ms % 20 == 0) {
      int n = 0;
      for (int i = 0; i < 6; i++) { if (fabs(cur[i] - prev[i]) > 1e-6) n++; prev[i] = cur[i]; }
      worst = std::max(worst, n);
    }
  }
  return worst;
}

int main() {
  setup();
  printf("one motor at a time\n");
  send("SPEED 80");
  send("ALL 40 60 80 30 -40 20");
  int worst = run(60000);
  CHECK(worst == 1, "ALL: never more than one joint moves in the same tick");
  CHECK(fabs(cur[0] - 40) < 0.2 && fabs(cur[1] - 60) < 0.2 && fabs(cur[2] - 80) < 0.2, "ALL: every joint reaches its target");
  send("HOME"); worst = run(60000);
  CHECK(worst == 1 && fabs(cur[1] - 90) < 0.2, "HOME: one joint at a time, ends at the home pose");

  printf("inverse kinematics\n");
  Serial.out.clear();
  send("XYZ 18 0 14 0 0 0"); worst = run(60000);
  CHECK(Serial.out.find("OK") != std::string::npos, "XYZ 18 0 14: accepted");
  CHECK(worst == 1, "XYZ: one joint at a time");
  CHECK(fabs(cur[1] - 39.2) < 0.3 && fabs(cur[2] - 59.9) < 0.3 && fabs(cur[4] + 20.7) < 0.3, "XYZ 18 0 14: shoulder 39.2, elbow 59.9, pitch -20.7");
  Serial.out.clear(); float before[6]; for (int i = 0; i < 6; i++) before[i] = tgt[i];
  send("XYZ 12 0 12 0 0 0");
  CHECK(Serial.out.find("ERR joint 2") != std::string::npos, "XYZ needing more elbow than allowed is rejected");
  bool same = true; for (int i = 0; i < 6; i++) if (tgt[i] != before[i]) same = false;
  CHECK(same, "a rejected XYZ changes no target");
  Serial.out.clear(); send("XYZ 40 0 14 0 0 0");
  CHECK(Serial.out.find("ERR out of reach") != std::string::npos, "XYZ out of reach is rejected");

  printf("stopping\n");
  send("HOME"); run(500); send("ALL 40 60 80 30 -40 20"); run(1500);
  send("STOP"); run(1500);
  float held[6]; for (int i = 0; i < 6; i++) held[i] = cur[i];
  worst = run(3000);
  bool still = true; for (int i = 0; i < 6; i++) if (fabs(cur[i] - held[i]) > 1e-6) still = false;
  CHECK(still && worst == 0, "STOP: the arm stays where it stopped");
  Serial.out.clear(); send("Q");
  CHECK(Serial.out.find("IDLE") != std::string::npos, "Q reports IDLE after STOP");

  printf("limits and guards\n");
  send("HOME"); run(60000);
  send("ANGLE 1 200"); CHECK(tgt[1] == J[1].maxJ, "targets beyond the limit are clamped");
  send("HOME"); run(60000);
  bool inside = true;
  for (int i = 0; i < 6; i++) for (float a = -1000; a <= 1000; a += 7)
    if (jointToServo(i, a) < SERVO_MIN_DEG || jointToServo(i, a) > SERVO_MAX_DEG) inside = false;
  CHECK(inside, "no servo is ever commanded into its end stops");
  send("SPEED 100"); CHECK(speedScale <= SPEED_CAP + 1e-6, "speed is capped");
  float t1 = tgt[1]; send("ALL 1 2 3"); CHECK(tgt[1] == t1, "an incomplete ALL command is ignored");
  send("RAW 0 1500"); send("HOME"); run(100);
  Serial.out.clear(); send("P");
  int fields = 0; { char *s = strdup(Serial.out.c_str()); for (char *t = strtok(s, " \n"); t; t = strtok(NULL, " \n")) fields++; free(s); }
  CHECK(fields == 7, "P prints 6 angles and the moving joint");

  printf("%s (%d failure%s)\n", failures ? "FIRMWARE TESTS FAILED" : "all firmware tests passed", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
