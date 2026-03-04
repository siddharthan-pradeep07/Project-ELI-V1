//                                             CODE BY SIDDHARTHAN (in partnership with claude and github co-pilot)
const int trigLeft  = 4,  echoLeft  = 2;
const int trigFront = 5,  echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 22, AIN2 = 23, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD    = 14;
const int FRONT_STOP        = 12;
const int FRONT_SATURATED   = 4;

const int BASE_SPEED        = 240;
const int TURN_TRIGGER_DIST = 3;

const int TURN_SPEED        = 240;
const int POST_TURN_NUDGE   = 900;
const int PRE_TURN_NUDGE    = 1050;
const int PRE_TURN_NUDGE_LEFT    = 300;
const int SPIN_90_MS        = 690;
const int SPIN_180_MS       = 1380;
const int POST_UTURN_BACK   = 1000;

const int KP             = 6;
const int MAX_CORRECTION = 60;
const int LEFT_TARGET    = 7;
const int RIGHT_TARGET   = 7;

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

void moveForwardSelfStabilize(long distLeft, long distRight) {
  int error = 0;
  bool hasLeft  = (distLeft  < WALL_THRESHOLD);
  bool hasRight = (distRight < WALL_THRESHOLD);
  if (hasLeft && hasRight)   error = (int)(distRight - distLeft);
  else if (hasLeft)          error = LEFT_TARGET  - (int)distLeft;
  else if (hasRight)         error = (int)distRight - RIGHT_TARGET;
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

bool isFrontBlocked(long d) {
  return (d <= FRONT_STOP) || (d <= FRONT_SATURATED);
}

// ---------------------------------------------------------------
// Single fast read — no median, used only for turn decisions
// ---------------------------------------------------------------
long quickDist(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 30000);
  return (dur == 0) ? 999 : dur * 0.034 / 2;
}

// ---------------------------------------------------------------
// At a junction, take 5 quick reads and return true if
// the majority (3+) show open.
// ---------------------------------------------------------------
bool isOpenMajority(int trigPin, int echoPin) {
  int openCount = 0;
  for (int i = 0; i < 5; i++) {
    if (quickDist(trigPin, echoPin) >= WALL_THRESHOLD) openCount++;
    delay(8);
  }
  return openCount >= 3;
}

// ---------------------------------------------------------------
// Turn routines
// ---------------------------------------------------------------
void turnLeft() {
  spinLeft(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

void turnRight() {
  spinRight(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

void turnAround() {
  spinRight(SPIN_180_MS);
  nudgeBack(POST_UTURN_BACK);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
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
  digitalWrite(13, HIGH); delay(500);
  digitalWrite(13, LOW);  delay(500);
}

// ---------------------------------------------------------------
// LOOP — strict left-hand wall following
// ---------------------------------------------------------------
void loop() {
  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distFront = getDistance(trigFront, echoFront);
  long distRight = getDistance(trigRight, echoRight);

  Serial.printf("L:%ld F:%ld R:%ld Lock:%d\n", distLeft, distFront, distRight, leftLockout);

  bool frontWall = isFrontBlocked(distFront);
  bool leftWall  = (distLeft  < WALL_THRESHOLD);
  bool rightWall = (distRight < WALL_THRESHOLD);

  if (leftLockout > 0) leftLockout--;

  // -------------------------------------------------------
  // 1. LEFT OPEN — highest priority (left-hand rule)
  //    Only fires when lockout has fully expired
  //
  //    THE ONLY CHANGE FROM ORIGINAL:
  //    stopMotors() + delay before the nudge, so that any
  //    steering correction from self-stabilize (which steers
  //    RIGHT toward the wall when left opens up) is fully
  //    killed before we nudge forward to the junction centre.
  //    Everything else — PRE_TURN_NUDGE value, turnLeft() —
  //    is identical to your original.
  // -------------------------------------------------------
  if (!leftWall && leftLockout == 0) {
    if (isOpenMajority(trigLeft, echoLeft)) {
      stopMotors();          // kill any steering inertia first
      delay(100);
      nudge(PRE_TURN_NUDGE_LEFT); // original value, unchanged
      turnLeft();
    } else {
      moveForwardSelfStabilize(distLeft, distRight);
    }
  }

  // -------------------------------------------------------
  // 2. FRONT CLEAR — go straight
  // -------------------------------------------------------
  else if (!frontWall) {
    moveForwardSelfStabilize(distLeft, distRight);
  }

  // -------------------------------------------------------
  // 3. FRONT BLOCKED — stop, nudge to junction centre,
  //    fresh majority read, U-turn is last resort.
  // -------------------------------------------------------
  else {
    stopMotors();
    delay(100);
    nudge(PRE_TURN_NUDGE);
    stopMotors();
    delay(150);

    bool leftOpen  = isOpenMajority(trigLeft,  echoLeft);
    bool rightOpen = isOpenMajority(trigRight, echoRight);
    bool frontOpen = !isFrontBlocked(getDistance(trigFront, echoFront));

    Serial.printf("  Junction: L=%d F=%d R=%d\n", leftOpen, frontOpen, rightOpen);

    if (leftOpen) {
      turnLeft();
    } else if (frontOpen) {
      leftLockout = LEFT_LOCKOUT_CYCLES;
      nudge(POST_TURN_NUDGE);
    } else if (rightOpen) {
      turnRight();
    } else {
      turnAround();
    }
  }

  delay(50);
}

