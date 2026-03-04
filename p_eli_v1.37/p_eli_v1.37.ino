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

const int BASE_SPEED        = 240;
const int TURN_TRIGGER_DIST = 3;   // cm

const int TURN_SPEED        = 240;
const int POST_TURN_NUDGE   = 900;
const int PRE_TURN_NUDGE    = 950;
const int SPIN_90_MS        = 750;
const int SPIN_180_MS       = 1500;
const int POST_UTURN_BACK   = 1000; // ms — reverse after U-turn to clear rear wall

// Stabilization tuning
const int KP                = 6;   // proportional gain — increase for stronger correction
const int MAX_CORRECTION    = 60;  // max speed delta applied to each motor
const int LEFT_TARGET       = 7;   // cm — desired distance from left wall (single-wall mode)
const int RIGHT_TARGET      = 7;   // cm — desired distance from right wall (single-wall mode)

// After any turn, suppress left-open detection for N loop cycles
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

void driveForward() {
  motorA(BASE_SPEED);
  motorB(BASE_SPEED);
}

// ---------------------------------------------------------------
// SELF STABILIZE
// Both walls : error = distRight - distLeft
// Left only  : error = LEFT_TARGET - distLeft
// Right only : error = distRight - RIGHT_TARGET
// No walls   : error = 0, drive straight
// positive correction → steer right, negative → steer left
// ---------------------------------------------------------------
void moveForwardSelfStabilize(long distLeft, long distRight) {
  int error = 0;

  bool hasLeft  = (distLeft  < WALL_THRESHOLD);
  bool hasRight = (distRight < WALL_THRESHOLD);

  if (hasLeft && hasRight) {
    error = (int)(distRight - distLeft);
  } else if (hasLeft) {
    error = LEFT_TARGET - (int)distLeft;
  } else if (hasRight) {
    error = (int)distRight - RIGHT_TARGET;
  }

  int correction = constrain(error * KP, -MAX_CORRECTION, MAX_CORRECTION);
  motorA(BASE_SPEED - correction);
  motorB(BASE_SPEED + correction);
}

void nudge(int ms) {
  driveForward();
  delay(ms);
  stopMotors();
  delay(80);
}

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
// Turn routines
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
  nudgeBack(POST_UTURN_BACK);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
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
// isFrontBlocked — wall detected OR sensor saturated (near zero)
// ---------------------------------------------------------------
bool isFrontBlocked(long distFront) {
  return (distFront <= FRONT_STOP) || (distFront <= FRONT_SATURATED);
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
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
}

// ---------------------------------------------------------------
// LOOP — strict left-hand wall following
//
// Priority (high → low):
//  1. Left open + lockout expired → turn left
//  2. Front clear                 → go straight (stabilized)
//  3. Front blocked, right open   → turn right
//  4. All blocked                 → turn around
// ---------------------------------------------------------------
void loop() {
  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distFront = getDistance(trigFront, echoFront);
  long distRight = getDistance(trigRight, echoRight);

  Serial.printf("L: %ld | F: %ld | R: %ld | Lockout: %d\n",
                distLeft, distFront, distRight, leftLockout);

  bool frontWall = isFrontBlocked(distFront);
  bool leftWall  = distLeft  < WALL_THRESHOLD;
  bool rightWall = distRight < WALL_THRESHOLD;

  if (leftLockout > 0) leftLockout--;

  // --- LEFT OPEN: turn left ---
  if (!leftWall && leftLockout == 0) {
    if (confirmLeftOpen()) {
      turnLeft();
    } else {
      moveForwardSelfStabilize(distLeft, distRight);
    }
  }

  // --- FRONT CLEAR: go straight with active stabilization ---
  else if (!frontWall) {
    moveForwardSelfStabilize(distLeft, distRight);
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