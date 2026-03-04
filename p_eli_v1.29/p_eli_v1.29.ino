const int trigLeft = 4, echoLeft = 2;
const int trigFront = 5, echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 22, AIN2 = 23, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD = 14;  // cm
const int FRONT_STOP = 12;      // cm
const int FRONT_WARN = 14;      // cm — only bail from self-stabilize if side wall is also lost
const int BASE_SPEED = 240;
const int TURN_TRIGGER_DIST = 3; // cm

const int TURN_SPEED = 240;
const int POST_TURN_NUDGE = 900;
const int PRE_TURN_NUDGE = 950;
const int SPIN_90_MS = 900;
const int SPIN_180_MS = 1800;

// Median filter for ultrasonic distance
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

// Motor control
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

void moveForward() {
  long distLeft  = getDistance(trigLeft, echoLeft);
  long distRight = getDistance(trigRight, echoRight);
  int correction = 0;
  if (distLeft != 999 && distRight != 999) {
    int error = distRight - distLeft;
    correction = constrain(error * 4, -40, 40);
  }
  motorA(BASE_SPEED - correction);
  motorB(BASE_SPEED + correction);
}

// ---------------------------------------------------------------
// FIX: Self-stabilize bails ONLY when a side wall is lost while
// the front is also getting close — true junction signal.
// Normal straight corridor with front wall ahead keeps going.
// ---------------------------------------------------------------
bool moveForwardSelfStabilize(long distLeft, long distRight) {
  long distFront = getDistance(trigFront, echoFront);

  if (distLeft < WALL_THRESHOLD && distRight < WALL_THRESHOLD) {
    // Both side walls present — check if a junction is forming:
    // side wall about to open up AND front is closing in
    bool frontClose = distFront <= FRONT_WARN;
    bool sideWallLoosening = (distLeft > (WALL_THRESHOLD - 4)) || (distRight > (WALL_THRESHOLD - 4));

    if (frontClose && sideWallLoosening) {
      // Junction detected — stop and let loop() handle the turn
      stopMotors();
      return false;
    }

    // Normal straight corridor — stabilize
    int error = distRight - distLeft;
    int correction = constrain(error * 4, -40, 40);
    motorA(BASE_SPEED - correction);
    motorB(BASE_SPEED + correction);
    return true;
  } else {
    // Side wall already lost — stop and let loop() handle the turn
    stopMotors();
    return false;
  }
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
}
void spinRight(int ms) {
  motorA(TURN_SPEED);
  motorB(-TURN_SPEED);
  delay(ms);
  stopMotors();
}

void turnLeft() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinLeft(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
}
void turnRight() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinRight(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
}
void turnAround() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinRight(SPIN_180_MS);
  nudge(POST_TURN_NUDGE);
}

bool confirmNoWall(int trigPin, int echoPin, int times = 3) {
  for (int i = 0; i < times; i++) {
    if (getDistance(trigPin, echoPin) < WALL_THRESHOLD) return false;
    delay(10);
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  pinMode(trigLeft, OUTPUT);  pinMode(echoLeft, INPUT);
  pinMode(trigFront, OUTPUT); pinMode(echoFront, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT); pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT); pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);
  delay(2000);
}

void loop() {
  static long prevLeft = 0, prevRight = 0;

  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distFront = getDistance(trigFront, echoFront);
  long distRight = getDistance(trigRight, echoRight);

  Serial.printf("L: %ld | F: %ld | R: %ld\n", distLeft, distFront, distRight);

  bool frontWall = distFront < FRONT_STOP;
  bool leftWall  = distLeft  < WALL_THRESHOLD;
  bool rightWall = distRight < WALL_THRESHOLD;

  // Detect sudden loss of left or right wall (for turn detection)
  bool lostLeft  = (prevLeft  < WALL_THRESHOLD && distLeft  >= WALL_THRESHOLD + 6);
  bool lostRight = (prevRight < WALL_THRESHOLD && distRight >= WALL_THRESHOLD + 6);
  prevLeft  = distLeft;
  prevRight = distRight;

  // ---------------------------------------------------------------
  // PRIORITY ORDER — front wall checked BEFORE self-stabilization
  // ---------------------------------------------------------------

  // 1. Front wall hit — must turn, takes absolute priority
  if (frontWall) {
    if (!rightWall) {
      turnRight();
    } else {
      turnAround();
    }
  }

  // 2. Sudden wall loss — junction detected mid-corridor
  else if (lostLeft) {
    nudge(PRE_TURN_NUDGE);
    moveForwardUntilFront(TURN_TRIGGER_DIST);
    spinLeft(SPIN_90_MS);
    nudge(POST_TURN_NUDGE);
  }
  else if (lostRight) {
    nudge(PRE_TURN_NUDGE);
    moveForwardUntilFront(TURN_TRIGGER_DIST);
    spinRight(SPIN_90_MS);
    nudge(POST_TURN_NUDGE);
  }

  // 3. No left wall — try to turn left (confirmed)
  else if (!leftWall && confirmNoWall(trigLeft, echoLeft)) {
    turnLeft();
  }

  // 4. Both side walls present and no front wall — self-stabilize
  //    but bail out if front gets too close (handled inside the function)
  else if (!frontWall && leftWall && rightWall) {
    moveForwardSelfStabilize(distLeft, distRight);
    // After bailing, loop() immediately re-evaluates on next iteration
  }

  // 5. Open corridor — just go forward
  else if (!frontWall) {
    moveForward();
  }

  delay(50);
}

