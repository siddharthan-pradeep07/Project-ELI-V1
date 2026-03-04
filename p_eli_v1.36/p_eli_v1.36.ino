//                                             CODE BY SIDDHARTHAN (in partnership with claude and github co-pilot)
const int trigLeft  = 4,  echoLeft  = 2;
const int trigFront = 5,  echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 22, AIN2 = 23, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD    = 14;  // cm — wall present if closer than this
const int FRONT_STOP        = 12;  // cm — front wall detected
const int FRONT_SATURATED   = 4;   // cm — sensor fully covered / touching wall
const int SIDE_SYMMETRY_TOL = 2;   // cm — L and R considered "equal" within this
const int SYMMETRY_COUNT    = 6;   // consecutive symmetric reads before backing up
const int BACK_ESCAPE_MS    = 600; // ms — how far to reverse on symmetric trap

const int BASE_SPEED        = 240;
const int TURN_TRIGGER_DIST = 3;   // cm

const int TURN_SPEED        = 240;
const int POST_TURN_NUDGE   = 900;
const int PRE_TURN_NUDGE    = 950;
const int SPIN_90_MS        = 750;
const int SPIN_180_MS       = 1500;
const int POST_UTURN_BACK   = 1000; // ms — reverse after U-turn to clear rear wall

// After any turn, suppress left-open detection for N loop cycles
// Each cycle ≈ 50ms delay + ~100ms sensor reads ≈ ~150ms
// 6 cycles ≈ ~900ms lockout
const int LEFT_LOCKOUT_CYCLES = 6;
int leftLockout    = 0;
int symmetryStreak = 0; // counts consecutive symmetric-sensor loops

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
// SELF STABILIZE — both walls or single left wall
// ---------------------------------------------------------------
void moveForwardSelfStabilize(long distLeft, long distRight) {
  int error = 0;
  if (distLeft < WALL_THRESHOLD && distRight < WALL_THRESHOLD) {
    error = (int)(distRight - distLeft);
  } else if (distLeft < WALL_THRESHOLD) {
    int target = WALL_THRESHOLD / 2;
    error = -(distLeft - target);
  }
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

// ---------------------------------------------------------------
// NUDGE BACK — reverse to clear a wall
// ---------------------------------------------------------------
void nudgeBack(int ms) {
  motorA(-BASE_SPEED);
  motorB(-BASE_SPEED);
  delay(ms);
  stopMotors();
  delay(80);
}

void moveForwardUntilFront(int targetDist) {
  while (true) {
    long d = getDistance(trigFront, echoFront);
    // Stop if close enough OR if sensor saturates (too close, returns near-zero)
    if (d <= targetDist || d <= FRONT_SATURATED) break;
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
// Turn routines — all reset leftLockout and symmetryStreak
// ---------------------------------------------------------------
void turnLeft() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinLeft(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout    = LEFT_LOCKOUT_CYCLES;
  symmetryStreak = 0;
}

void turnRight() {
  nudge(PRE_TURN_NUDGE);
  spinRight(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout    = LEFT_LOCKOUT_CYCLES;
  symmetryStreak = 0;
}

void turnAround() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinRight(SPIN_180_MS);
  nudgeBack(POST_UTURN_BACK);   // reverse to gain clearance after 180°
  nudge(POST_TURN_NUDGE);
  leftLockout    = LEFT_LOCKOUT_CYCLES;
  symmetryStreak = 0;
}

// ---------------------------------------------------------------
// Confirm left is genuinely open — 2 reads, noise guard
// ---------------------------------------------------------------
bool confirmLeftOpen() {
  for (int i = 0; i < 2; i++) {
    if (getDistance(trigLeft, echoLeft) < WALL_THRESHOLD) return false;
    delay(10);
  }
  return true;
}

// ---------------------------------------------------------------
// isFrontBlocked — true if sensor reads wall OR is saturated
// (fully covered sensor returns very small values near 0)
// ---------------------------------------------------------------
bool isFrontBlocked(long distFront) {
  return (distFront <= FRONT_STOP) || (distFront <= FRONT_SATURATED);
}

// ---------------------------------------------------------------
// checkSymmetryTrap — detects the mouse is stuck going straight
// while L and R distances are equal (corridor with no side variation)
// AND the front sensor is unusually high (saturated the other way,
// i.e. open-air misread while actually heading into a wall).
//
// Condition: both side walls present, |distL - distR| <= tolerance,
// front reads suspiciously high (> WALL_THRESHOLD * 3),
// persisting for SYMMETRY_COUNT consecutive loops.
// ---------------------------------------------------------------
bool checkSymmetryTrap(long distLeft, long distFront, long distRight) {
  bool sidesSymmetric = (distLeft  < WALL_THRESHOLD) &&
                        (distRight < WALL_THRESHOLD) &&
                        (abs(distLeft - distRight) <= SIDE_SYMMETRY_TOL);
  bool frontSuspicious = (distFront > WALL_THRESHOLD * 3); // looks open but may be saturated high

  if (sidesSymmetric && frontSuspicious) {
    symmetryStreak++;
  } else {
    symmetryStreak = 0;
  }
  return (symmetryStreak >= SYMMETRY_COUNT);
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
  pinMode(13, OUTPUT);
  digitalWrite(STBY, HIGH);
  delay(2000);
  digitalWrite(13, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                      // wait for a second
  digitalWrite(13, LOW);   // turn the LED off by making the voltage LOW
  delay(500); 
}

// ---------------------------------------------------------------
// LOOP — strict left-hand wall following
//
// Priority (high → low):
//  1. Symmetry trap (stuck straight, suspicious front) → back up
//  2. Left open + lockout expired                      → turn left
//  3. Front clear                                      → go straight (stabilized)
//  4. Front blocked, right open                        → turn right
//  5. All blocked                                      → turn around
// ---------------------------------------------------------------
void loop() {
  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distFront = getDistance(trigFront, echoFront);
  long distRight = getDistance(trigRight, echoRight);

  Serial.printf("L: %ld | F: %ld | R: %ld | Lockout: %d | Streak: %d\n",
                distLeft, distFront, distRight, leftLockout, symmetryStreak);

  bool frontWall = isFrontBlocked(distFront);
  bool leftWall  = distLeft  < WALL_THRESHOLD;
  bool rightWall = distRight < WALL_THRESHOLD;

  if (leftLockout > 0) leftLockout--;

  // --- SYMMETRY TRAP: back up and reassess ---
  if (checkSymmetryTrap(distLeft, distFront, distRight)) {
    Serial.println(">> Symmetry trap! Backing up.");
    nudgeBack(BACK_ESCAPE_MS);
    symmetryStreak = 0;
    leftLockout    = LEFT_LOCKOUT_CYCLES; // re-suppress left snap after reversing
  }

  // --- LEFT OPEN: turn left (highest nav priority, left-hand rule) ---
  // FIX: was `leftLockout > 0` (wrong — fired during lockout, suppressed after)
  //      corrected to `leftLockout == 0` (fire only after lockout expires)
  else if (!leftWall && leftLockout == 0) {
    if (confirmLeftOpen()) {
      turnLeft();
    } else {
      moveForwardSelfStabilize(distLeft, distRight);
    }
  }

  // --- FRONT CLEAR: go straight ---
  else if (!frontWall) {
    if (leftWall) {
      moveForwardSelfStabilize(distLeft, distRight);
    } else {
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
