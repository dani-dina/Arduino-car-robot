#define ENA 10
#define IN1 9 
#define IN2 8 
#define IN3 7 
#define IN4 6  
#define ENB 5 

#define LEFT_IR A0
#define RIGHT_IR A1
#define ECHO_PIN A2    
#define TRIG_PIN A3 
#define SERVO_PIN A5

const int OBSTACLE_THRESHOLD = 15;
const int MOTOR_SPEED = 200;
const int SERVO_MIN_ANGLE = 0;
const int SERVO_MAX_ANGLE = 140;
const int SERVO_CENTER = 70;
const int SERVO_STEP = 5;

int distanceLeft = 0, distanceFront = 0, distanceRight = 0;

void setup() { 
  Serial.begin(9600);

  pinMode(LEFT_IR, INPUT);  
  pinMode(RIGHT_IR, INPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT); 
  pinMode(IN2, OUTPUT); 
  pinMode(IN3, OUTPUT);   
  pinMode(IN4, OUTPUT); 
  pinMode(ENB, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);

  analogWrite(ENA, MOTOR_SPEED);
  analogWrite(ENB, MOTOR_SPEED);

  centerServo();

  distanceFront = readUltrasonic();
  delay(500);
}

void loop() {  
  distanceFront = readUltrasonic();
  Serial.print("Front Distance: "); Serial.println(distanceFront);

  int leftIR = digitalRead(LEFT_IR);
  int rightIR = digitalRead(RIGHT_IR);

  if (leftIR == 0 && rightIR == 0) {
    if (distanceFront > OBSTACLE_THRESHOLD) {
      moveForward();
    } else {
      checkSides();
    }
  } 
  else if (leftIR == 0 && rightIR == 1) {
    turnLeft();
  } 
  else if (leftIR == 1 && rightIR == 0) {
    turnRight();
  }

  delay(10);
}

// ======== Movement Functions =========
void moveForward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ======== Servo Control =========
void centerServo() {
  moveServoSmooth(SERVO_CENTER);
}

void moveServoSmooth(int targetAngle) {
  int pwm = (targetAngle * 11) + 500;
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pwm);
  digitalWrite(SERVO_PIN, LOW);
  delay(50);
}

// ======== Ultrasonic Distance =========
int readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration / 29 / 2;
  return constrain(distance, 0, 300); // limit range
}

// ======== Obstacle Handling =========
void checkSides() {
  stopMotors();
  delay(100);

  // Scan right
  for (int angle = SERVO_CENTER; angle <= SERVO_MAX_ANGLE; angle += SERVO_STEP) {
    moveServoSmooth(angle);
  }
  delay(300);
  distanceRight = readUltrasonic();
  Serial.print("Right Distance: "); Serial.println(distanceRight);
  delay(100);

  // Scan left
  for (int angle = SERVO_MAX_ANGLE; angle >= SERVO_MIN_ANGLE; angle -= SERVO_STEP) {
    moveServoSmooth(angle);
  }
  delay(300);
  distanceLeft = readUltrasonic();
  Serial.print("Left Distance: "); Serial.println(distanceLeft);
  delay(100);

  // Return to center
  for (int angle = SERVO_MIN_ANGLE; angle <= SERVO_CENTER; angle += SERVO_STEP) {
    moveServoSmooth(angle);
  }

  compareAndTurn();
}

// ======== Decide Direction =========
void compareAndTurn() {
  if (distanceLeft > distanceRight) {
    turnLeft();
    delay(500);
    moveForward(); delay(600);
    turnRight(); delay(500);
    moveForward(); delay(600);
    turnRight(); delay(400);
  } else {
    turnRight();
    delay(500);
    moveForward(); delay(600);
    turnLeft(); delay(500);
    moveForward(); delay(600);
    turnLeft(); delay(400);
  }
}
