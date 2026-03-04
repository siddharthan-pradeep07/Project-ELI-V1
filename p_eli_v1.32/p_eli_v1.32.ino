const int trigLeft = 4, echoLeft = 2;
const int trigFront = 5, echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 22, AIN2 = 23, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD  = 14;  // cm — wall detected if closer than this
const int FRONT_STOP      = 12;  // cm — stop and turn if front closer than this
const int WALL_FOLLOW_DIST = 7;  // cm — desired distance to hug left wall
const int BASE_SPEED      = 240;
const int TURN_TRIGGER_DIST = 3; // cm

const int TURN_SPEED      = 240;
const int POST_TURN_NUDGE = 900; // ms
const int PRE_TURN_NUDGE  = 950; // ms
const int SPIN_90_MS      = 750;
const int SPIN_180_MS     = 1500;

// Lockout to suppress false left-turn re-trigger after any turn completes
const int LEFT_LOCKOUT_CYCLES = 5;
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
// moveForward — 3 modes depending on which walls are visible:
//
//  Both walls  → differential correction (center the robot)
//  Left only   → hug left wall at WALL_FOLLOW_DIST (ignore missing right)
//  Neither     → drive straight, no correction
// ---------------------------------------------------------------
void moveForward() {
  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distRight = getDistance(trigRight, echoRight);

  bool hasLeft  = (distLeft  < WALL_THRESHOLD);
  bool hasRight = (distRight < WALL_THRESHOLD);
  int correction = 0;

  if (hasLeft && hasRight) {
    // Both walls — center between them
    int error = distRight - distLeft;
    correction = constrain(error * 4, -40, 40);
  }
  else if (hasLeft) {
    // Left wall only — maintain fixed distance from left wall
    // Positive error = too far from left = steer left (negative correction)
    int error = distLeft - WALL_FOLLOW_DIST;
    correction = constrain(error * 5, -40, 40);
    // correction > 0 means right drift → reduce motorA, increase motorB = steer left
  }
  // Right wall only or no walls: drive straight (correction stays 0)

  motorA(BASE_SPEED - correction);
  motorB(BASE_SPEED + correction);
}

void nudge(int ms) {
  moveForward();
  delay(ms);
  stopMotors();
  delay(80);
}

void moveForwardUntilFront(int targetDist) {
  while (getDistance(trigFront, echoFront) > targetDist) {
    moveForward();
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
// Turn functions
// NOTE: turnRight does NOT call moveForwardUntilFront — the robot
// is already correctly positioned when the front wall triggers.
// Calling moveForwardUntilFront here caused it to creep into the
// intersection corner and graze the wall.
// ---------------------------------------------------------------
void turnLeft() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinLeft(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

void turnRight() {
  // Just nudge forward to clear the intersection, then spin — no moveForwardUntilFront
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

// Confirm wall is truly absent across N reads
bool confirmNoWall(int trigPin, int echoPin, int times = 4) {
  for (int i = 0; i < times; i++) {
    if (getDistance(trigPin, echoPin) < WALL_THRESHOLD) return false;
    delay(15);
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
// -----|-------|-------|-----------------------------
//  0   |   x   |   x   | Turn Left  (lockout permitting)
//  1   |   0   |   x   | Go Straight (left wall, front clear)
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

  if (!leftWall && leftLockout == 0) {
    // Left open — confirm it's real then turn left
    if (confirmNoWall(trigLeft, echoLeft)) {
      turnLeft();
    } else {
      moveForward();
    }
  }
  else if (!frontWall) {
    // Front clear — go straight using left-wall hugging
    // This correctly handles: straight only, straight+right
    moveForward();
  }
  else if (!rightWall) {
    // Front blocked, right open — turn right
    turnRight();
  }
  else {
    // All blocked — dead end
    turnAround();
  }

  delay(50);
}

