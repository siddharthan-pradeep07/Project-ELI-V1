const int trigLeft = 4, echoLeft = 2;
const int trigFront = 5, echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 22, AIN2 = 23, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD = 14;  // cm
const int FRONT_STOP     = 12;  // cm
const int BASE_SPEED     = 240;
const int TURN_TRIGGER_DIST = 3; // cm

const int TURN_SPEED      = 240;
const int POST_TURN_NUDGE = 900; // ms
const int PRE_TURN_NUDGE  = 950; // ms
const int SPIN_90_MS      = 900;
const int SPIN_180_MS     = 1800;

// How many loop() cycles to ignore left-open after any turn.
// Each cycle ~50ms delay + ~200ms sensor time ≈ 250ms per cycle.
// 5 cycles ≈ ~1.25 seconds of lockout — enough to re-acquire walls cleanly.
const int LEFT_LOCKOUT_CYCLES = 5;
int leftLockout = 0; // counts down each loop()

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

void moveForward() {
  long distLeft  = getDistance(trigLeft,  echoLeft);
  long distRight = getDistance(trigRight, echoRight);
  int correction = 0;
  if (distLeft != 999 && distRight != 999) {
    int error = distRight - distLeft;
    correction = constrain(error * 4, -40, 40);
  }
  motorA(BASE_SPEED - correction);
  motorB(BASE_SPEED + correction);
}

void moveForwardSelfStabilize(long distLeft, long distRight) {
  if (distLeft < WALL_THRESHOLD && distRight < WALL_THRESHOLD) {
    int error = distRight - distLeft;
    int correction = constrain(error * 4, -40, 40);
    motorA(BASE_SPEED - correction);
    motorB(BASE_SPEED + correction);
  } else {
    stopMotors();
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
// Turn functions — all reset lockout on completion so the robot
// doesn't misread the newly-opened left side as a left-turn trigger
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
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinRight(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES; // KEY: suppresses false left snap after right turn
}

void turnAround() {
  nudge(PRE_TURN_NUDGE);
  moveForwardUntilFront(TURN_TRIGGER_DIST);
  spinRight(SPIN_180_MS);
  nudge(POST_TURN_NUDGE);
  leftLockout = LEFT_LOCKOUT_CYCLES;
}

// Confirm wall is truly absent — re-reads N times with small delay
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
// Decision table:
// Left | Front | Right | Action
// -----|-------|-------|--------------------------------
//  0   |   x   |   x   | Turn Left  (blocked if locked out)
//  1   |   0   |   x   | Go Straight (front clear = priority)
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

  // Tick down lockout each cycle
  if (leftLockout > 0) leftLockout--;

  if (!leftWall && leftLockout == 0) {
    // Left genuinely open and lockout expired — confirm then turn
    if (confirmNoWall(trigLeft, echoLeft)) {
      turnLeft();
    } else {
      moveForward(); // noise glitch, keep straight
    }
  }
  else if (!frontWall) {
    // Left wall present (or locked out), front clear — go straight
    if (leftWall && rightWall) {
      moveForwardSelfStabilize(distLeft, distRight);
    } else {
      moveForward();
    }
  }
  else if (!rightWall) {
    // Left + front blocked, right open — turn right
    turnRight();
  }
  else {
    // All walls — dead end
    turnAround();
  }

  delay(50);
}
