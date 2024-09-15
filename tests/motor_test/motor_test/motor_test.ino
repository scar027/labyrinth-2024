// Motor control pins
#define MOTOR_LEFT_FORWARD 23
#define MOTOR_LEFT_BACKWARD 21
#define MOTOR_RIGHT_FORWARD 32
#define MOTOR_RIGHT_BACKWARD 33
#define ENABLE_LEFT 13
#define ENABLE_RIGHT 14

const int MOTOR_SPEED = 100;

// Motor control functions
void moveForward() {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(ENABLE_LEFT, MOTOR_SPEED);  // Set speed for left motor
    analogWrite(ENABLE_RIGHT, MOTOR_SPEED); // Set speed for right motor
}

void stopRobot() {
    analogWrite(ENABLE_LEFT, 0);  // Set speed for turning
    analogWrite(ENABLE_RIGHT, 0);
}

void turnRight() {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    analogWrite(ENABLE_LEFT, MOTOR_SPEED);  // Set speed for turning
    analogWrite(ENABLE_RIGHT, MOTOR_SPEED);
    delay(700);
}

void turnLeft() {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(ENABLE_LEFT, MOTOR_SPEED);  // Set speed for turning
    analogWrite(ENABLE_RIGHT, MOTOR_SPEED);
    delay(700);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
    
  // Initialize motor pins
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);
  pinMode(ENABLE_LEFT, OUTPUT);  // Set enable pin as output
  pinMode(ENABLE_RIGHT, OUTPUT);

}

void loop()
{
  // moveForward();
  // turnLeft();
  // turnRight();
  stopRobot();
  delay(10);
}