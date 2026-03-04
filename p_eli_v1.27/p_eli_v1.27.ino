const int trigLeft = 4, echoLeft = 2;
const int trigFront = 5, echoFront = 18;
const int trigRight = 19, echoRight = 21;
const int STBY = 15;
const int AIN1 = 23, AIN2 = 22, PWMA = 27;
const int BIN1 = 33, BIN2 = 32, PWMB = 14;

const int WALL_THRESHOLD = 14;  // cm
const int FRONT_STOP = 12;      // cm
const int BASE_SPEED = 240;
const int TURN_TRIGGER_DIST = 3; // cm

const int TURN_SPEED = 240;
const int POST_TURN_NUDGE = 900;
const int PRE_TURN_NUDGE = 600; // ms, further increased pre-nudge before turning
const int SPIN_90_MS = 800;
const int SPIN_180_MS = 1600;

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
  long distLeft = getDistance(trigLeft, echoLeft);
  long distRight = getDistance(trigRight, echoRight);
  int correction = 0;
  if (distLeft != 999 && distRight != 999) {
    int error = distRight - distLeft;
    correction = constrain(error * 4, -40, 40);
  }
  // Always move forward, only adjust if both sensors are valid
  motorA(BASE_SPEED - correction);
  motorB(BASE_SPEED + correction);
}
void moveForwardSelfStabilize(long distLeft, long distRight) {
  // Only self-stabilize if both walls are detected
  if (distLeft < WALL_THRESHOLD && distRight < WALL_THRESHOLD) {
    int error = distRight - distLeft;
    int correction = constrain(error * 4, -40, 40);
    motorA(BASE_SPEED - correction);
    motorB(BASE_SPEED + correction);
  } else {
    // If either wall is lost, stop and let loop() handle the turn
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
  // Corrected: left turn should be left wheel backward, right wheel forward
  motorA(-TURN_SPEED);
  motorB(TURN_SPEED);
  delay(ms);
  stopMotors();
}
void spinRight(int ms) {
  // Corrected: right turn should be right wheel backward, left wheel forward
  motorA(TURN_SPEED);
  motorB(-TURN_SPEED);
  delay(ms);
  stopMotors();
}
void turnLeft() {
  nudge(PRE_TURN_NUDGE); // pre-nudge before turning (not affected by front wall)
  spinLeft(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
}
void turnRight() {
  nudge(PRE_TURN_NUDGE); // pre-nudge before turning (not affected by front wall)
  spinRight(SPIN_90_MS);
  nudge(POST_TURN_NUDGE);
}
void turnAround() {
  nudge(PRE_TURN_NUDGE); // pre-nudge before turning (not affected by front wall)
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
  bool lostLeft = (prevLeft < WALL_THRESHOLD && distLeft >= WALL_THRESHOLD + 6);
  bool lostRight = (prevRight < WALL_THRESHOLD && distRight >= WALL_THRESHOLD + 6);
  prevLeft = distLeft;
  prevRight = distRight;

  // LEFT WALL FOLLOWING with immediate turn on wall loss:
  if (lostLeft) {
    nudge(PRE_TURN_NUDGE);
    spinLeft(SPIN_90_MS);
    nudge(POST_TURN_NUDGE);
  }
  else if (lostRight) {
    nudge(PRE_TURN_NUDGE);
    spinRight(SPIN_90_MS);
    nudge(POST_TURN_NUDGE);
  }
  else if (!leftWall && confirmNoWall(trigLeft, echoLeft)) {
    turnLeft();
  }
  else if (!frontWall && leftWall && rightWall) {
    moveForwardSelfStabilize(distLeft, distRight);
  }
  else if (!frontWall) {
    moveForward();
  }
  else if (!rightWall) {
    turnRight();
  }
  else {
    turnAround();
  }
  delay(50);
}
