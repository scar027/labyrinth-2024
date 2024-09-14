#include <HCSR04.h>

// Define pin connections for ultrasonic sensors
#define TRIG_RIGHT 25
#define ECHO_RIGHT 22
#define TRIG_FRONT 26
#define ECHO_FRONT 27

// Motor control pins
#define MOTOR_LEFT_FORWARD 23
#define MOTOR_LEFT_BACKWARD 21
#define MOTOR_RIGHT_FORWARD 32
#define MOTOR_RIGHT_BACKWARD 33
#define ENABLE_LEFT 13
#define ENABLE_RIGHT 14

// Threshold for detecting walls (in cm)
const int WALL_THRESHOLD = 10;
const int RIGHT_WALL_THRESHOLD = 25;
const int MOTOR_SPEED = 100;


// Ultrasonic sensors setup
UltraSonicDistanceSensor rightSensor(TRIG_RIGHT, ECHO_RIGHT);
UltraSonicDistanceSensor frontSensor(TRIG_FRONT, ECHO_FRONT);

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

void turnRight(p_val_motor_speed) {
    digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
    digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
    digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
    analogWrite(ENABLE_LEFT, p_val_motor_speed);  // Set speed for turning
    analogWrite(ENABLE_RIGHT, p_val_motor_speed);
    // delay(500);
}

void turnLeft(p_val_motor_speed) {
    digitalWrite(MOTOR_LEFT_FORWARD, LOW);
    digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
    digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
    digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
    analogWrite(ENABLE_LEFT, p_val_motor_speed );  // Set speed for turning
    analogWrite(ENABLE_RIGHT, p_val_motor_speed);
    // delay(500);
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

void loop() {
  // put your main code here, to run repeatedly:
  
  // right wall following robot log :
  // state1: wall on right and nothing ahead : follow wall
  // state2: wall on right and obstacle ahead : turn left
  // state3: no wall on right : turn right

  // Get distance readings from sensors
  int rightDistance = rightSensor.measureDistanceCm();
  int frontDistance = frontSensor.measureDistanceCm();

  if (rightDistance <= 0) rightDistance = 0;
  if (frontDistance <= 0) frontDistance = 0;

  Serial.print("Right: "); Serial.print(rightDistance);
  Serial.print(" | Front: "); Serial.println(frontDistance);

  if (rightDistance <= RIGHT_WALL_THRESHOLD && frontDistance >= WALL_THRESHOLD){
      moveForward();
      Serial.println("Case 1"); // print statements for debugging
  }
  else if (rightDistance <= RIGHT_WALL_THRESHOLD && frontDistance <= WALL_THRESHOLD){
      stopRobot();
      delay(1000);
      output_pwm = MOTOR_SPEED * (10 / frontDistance);
      if (output_pwm > 50){
        turnLeft(output_pwm);
      }
      Serial.println("Case 2");
  }
  else if (rightDistance > RIGHT_WALL_THRESHOLD){
      stopRobot();
      delay(1000);
      output_pwm = MOTOR_SPEED * (10/frontDistance);
    if (output_pwm > 50){
        turnRight(output_pwm);
      }
      Serial.println("Case 3");
  }

  delay(10);
}