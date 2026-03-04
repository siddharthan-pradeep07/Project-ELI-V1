const int trigLeft  = 4,  echoLeft  = 2;
const int trigFront = 5,  echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 22, AIN2 = 23, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD   = 14; // cm — wall present if closer than this
const int FRONT_STOP       = 12; // cm — front wall detected

const int BASE_SPEED       = 240;
const int TURN_TRIGGER_DIST = 3; // cm

const int TURN_SPEED      = 240;
const int POST_TURN_NUDGE = 900;
const int PRE_TURN_NUDGE  = 950;
const int SPIN_90_MS      = 800;
const int SPIN_180_MS     = 1800;

// After any turn, suppress left-open detection for N loop cycles
// to avoid false left-snap into the wall we just came from.
// Each cycle ≈ 50ms delay + ~100ms sensor reads ≈ ~150ms
// 6 cycles ≈ ~900ms lockout
const int LEFT_LOCKOUT_CYCLES = 6;
int leftLockout = 0;

// ---------------------------------------------------------------
// Median filter ultrasonic read
// ---------------------------------------------------------------
long getDistance(int trigPin, int echoPin) {
  long readings[5];
  for (int i = 0; i < 5; i++) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH, 30000);
    readings[i] = (duration == 0) ? 999 : duration * 0.034 / 2;
    delay(5);
  }
  for (int i = 0; i < 4; i++)
    for (int j = i + 1; j < 5; j++)
      if (readings[j] < readings[i]) {
        long tmp = readings[i];
        readings[i] = readings[j];
        readings[j] = tmp;
      }
  return readings[2];
}

// ---------------------------------------------------------------
// Motor control
// ---------------------------------------------------------------
void motorA(int speed) {
  digitalWrite(AIN1, speed > 0 ? HIGH : LOW);
  digitalWrite(AIN2, speed > 0 ? LOW : HIGH);
  analogWrite(PWMA, abs(speed));
}
void motorB(int speed) {
  digitalWrite(BIN1, speed > 0 ? HIGH : LOW);
  digitalWrite(BIN2, speed > 0 ? LOW : HIGH);
  analogWrite(PWMB, abs(speed));
}
void stopMotors() { motorA(0); motorB(0); }

// ---------------------------------------------------------------
// DRIVE STRAIGHT — no correction, pure forward
// ---------------------------------------------------------------
void driveForward() {
  motorA(BASE_SPEED);
  motorB(BASE_SPEED);
}

// ---------------------------------------------------------------
// SELF STABILIZE — works for both walls or single left wall
//
// error = distRight - distLeft
//   positive (distRight > distLeft) = robot too close to left wall
//     → correction > 0 → motorA gets less, motorB gets more → steers RIGHT (away from left)
//   negative (distLeft > distRight) = robot too close to right wall
//     → correction < 0 → motorA gets more, motorB gets less → steers LEFT (away from right)
//
// Single wall (right missing, distRight = 999):
//   We can't use 999 directly — it would give a huge false correction.
//   Instead mirror distLeft so error = distLeft - distLeft = 0 as baseline,
//   then apply a gentle correction based on distLeft alone to keep it centred.
//   If distLeft is small (too close to left) → steer right.
//   If distLeft is large (drifting away from left) → steer left.
//   Target: keep distLeft near half of WALL_THRESHOLD (7 cm).
// ---------------------------------------------------------------
void moveForwardSelfStabilize(long distLeft, long distRight) {
  int error = 0;

  if (distLeft < WALL_THRESHOLD && distRight < WALL_THRESHOLD) {
    // Both walls — pure differential, most accurate
    error = (int)(distRight - distLeft);
  } else if (distLeft < WALL_THRESHOLD) {
    // Left wall only — use distLeft deviation from target (half threshold)
    int target = WALL_THRESHOLD / 2; // 7 cm
    error = -(distLeft - target);    // too close → positive error → steer right
  }
  // No walls: error stays 0, drive straight

  int correction = constrain(error * 4, -40, 40);
  motorA(BASE_SPEED - correction);
  motorB(BASE_SPEED + correction);
}

void nudge(int ms) {
  driveForward();
  delay(ms);
  stopMotors();
  delay(80);
}

void moveForwardUntilFront(int targetDist) {
  while (getDistance(trigFront, echoFront) > targetDist) {
    driveForward();
    delay(10);
  }
  stopMotors();
  delay(80);
}

void spinLeft(int ms) {
  motorA(-TURN_SPEED);
  motorB(TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(100);
}
void spinRight(int ms) {
  motorA(TURN_SPEED);
  motorB(-TURN_SPEED);
  delay(ms);
  stopMotors();
  delay(100);
}

// ---------------------------------------------------------------
// turnLeft  — nudge into junction, creep to front ref, spin, nudge out
// turnRight — nudge to clear intersection corner, spin, nudge out
//             (no moveForwardUntilFront — avoids grazing corner wall)
// turnAround — nudge, creep to front, 180, nudge out
// All turns reset leftLockout to prevent false left re-trigger
// ---------------------------------------------------------------
void turnLeft() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinLeft(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

void turnRight() {
  nudge(PRE_TURN_NUDGE);
  spinRight(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

void turnAround() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinRight(SPIN_180_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

// ---------------------------------------------------------------
// Confirm left is genuinely open — 2 reads only so we don't
// overshoot the junction before acting on it
// ---------------------------------------------------------------
bool confirmLeftOpen() {
  for (int i = 0; i < 2; i++) {
    if (getDistance(trigLeft, echoLeft) < WALL_THRESHOLD) return false;
    delay(10);
  }
  return true;
}

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(trigLeft,  OUTPUT); pinMode(echoLeft,  INPUT);
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
  delay(2000);
}

// ---------------------------------------------------------------
// LOOP — strict left-hand wall following
//
// Left | Front | Right | Action
// -----|-------|-------|-------------------------------------
//  0   |   x   |   x   | Turn Left  (if lockout expired)
//  1   |   0   |   1   | Self-stabilize (both walls, go straight)
//  1   |   0   |   0   | Hug left wall  (right open, go straight)
//  1   |   1   |   0   | Turn Right
//  1   |   1   |   1   | Turn Around
// ---------------------------------------------------------------
void loop() {
  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distFront = getDistance(trigFront, echoFront);
  long distRight = getDistance(trigRight, echoRight);

  Serial.printf("L: %ld | F: %ld | R: %ld | Lockout: %d\n",
                distLeft, distFront, distRight, leftLockout);

  bool frontWall = distFront < FRONT_STOP;
  bool leftWall  = distLeft  < WALL_THRESHOLD;
  bool rightWall = distRight < WALL_THRESHOLD;

  if (leftLockout > 0) leftLockout--;

  // --- LEFT OPEN: turn left (highest priority, left-hand rule) ---
  if (!leftWall && leftLockout > 0) {
    if (confirmLeftOpen()) {
      turnLeft();
    } else {
      // Noise — stabilize using whatever walls are visible
      moveForwardSelfStabilize(distLeft, distRight);
    }
  }

  // --- FRONT CLEAR: go straight ---
  else if (!frontWall) {
    if (leftWall) {
      // Left wall present (with or without right) — self-stabilize handles both cases
      moveForwardSelfStabilize(distLeft, distRight);
    } else {
      // No wall reference at all — drive straight
      driveForward();
    }
  }

  // --- FRONT BLOCKED, RIGHT OPEN: turn right ---
  else if (!rightWall) {
    turnRight();
  }

  // --- ALL BLOCKED: dead end ---
  else {
    turnAround();
  }

  delay(50);
}

