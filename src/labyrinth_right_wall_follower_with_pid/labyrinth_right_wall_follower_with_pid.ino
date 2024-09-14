#include <math.h>
#include <HCSR04.h>


// ultrasonic sensor setup
#define TRIG_RIGHT 25
#define ECHO_RIGHT 22
#define TRIG_FRONT 26
#define ECHO_FRONT 27

// motor driver setup
#define EN_R 14
#define RM1 32
#define RM2 33
#define EN_L 13
#define LM1 23
#define LM2 21

// minimum and maximum speed of the motors
const int MIN_SPEED = 0;
const int MAX_SPEED = 127;


// Ultrasonic sensors setup
UltraSonicDistanceSensor rightSensor(TRIG_RIGHT, ECHO_RIGHT);
UltraSonicDistanceSensor frontSensor(TRIG_FRONT, ECHO_FRONT);

// Front and left reference points
const float SETPOINT_FRONT = 12.0; //cm
const float SETPOINT_RIGHT = 35.0; //cm

// PID constants
const float KP_FORWARD = 4.0;
const float KI_FORWARD = 0.0;
const float KD_FORWARD = 36.0;
const float KP_RIGHT = 5.0;
const float KI_RIGHT = 0.0;
const float KD_RIGHT = 40.0;

// Domain initialization
bool initForward = true;
bool initRight = true;

// PID variables
float error = 0.0;
float integral = 0.0;
float derivative = 0.0;
float lastError = 0.0;

// Value of PID
int pidValue = 0;

// Motor control functions
void moveForward(int LMS, int RMS) {

  // LMS = (LMS/255) * MAX_SPEED;
  // RMS = (RMS/255) * MAX_SPEED;
  Serial.println(LMS);
  Serial.println(RMS);
  digitalWrite(LM1, HIGH);
  digitalWrite(RM1, HIGH);
  digitalWrite(LM2, LOW);
  digitalWrite(RM2, LOW);
  analogWrite(EN_L, LMS); // Set speed for left motor
  analogWrite(EN_R, RMS); // Set speed for right motor
  delay(200);
}

void stopRobot() {
  analogWrite(EN_L, 0);  // Set speed for turning
  analogWrite(EN_R, 0);
}


void turnLeft() {
  digitalWrite(LM1, LOW);
  digitalWrite(RM1, HIGH);
  digitalWrite(LM2, HIGH);
  digitalWrite(RM2, LOW);
  analogWrite(EN_L, MAX_SPEED);  // Set speed for turning
  analogWrite(EN_R, MAX_SPEED);
}

int fitCorrection(int val)
{
  if (val > 127 && val > 0)
  { 
    val = 127;
  }
  if (val < -127 && val < 0)
  {
    val = -127;
  }
  return val;
}

void resetPIDVar()
{
  integral = 0;
  derivative = 0;
  lastError = 0;
}


void setup() {
  Serial.begin(115200);
    
  // Initialize motor pins
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);
  pinMode(EN_L, OUTPUT);  // Set enable pin as output
  pinMode(EN_R, OUTPUT);
}

void loop ()
{ 
  // Get distance readings from sensors
  int rightDistance = rightSensor.measureDistanceCm();
  int frontDistance = frontSensor.measureDistanceCm();

  if (rightDistance <= 0) rightDistance = 0;
  if (frontDistance <= 0) frontDistance = 0;

  Serial.print("Front: "); Serial.print(frontDistance);
  Serial.print(" | Right: "); Serial.println(rightDistance);

  // If the front sensor detects the wall at the front reference point
  if (frontDistance < SETPOINT_FRONT)
  {
    initForward= true;
    initRight  = true;
    // the robot will turn left
    turnLeft();
    delay(270);
    // the robot stops for a moment
    stopRobot();
    delay(70);
  }
  // if the front sensor does not detect the wall at the front reference point
  else 
  { // Calculate error with respect to the wall
    error = rightDistance - SETPOINT_RIGHT;

    // Calculate PID variables
    integral = error + integral;
    derivative = error - lastError;
    lastError = error;

    if (error < 20) // when the error is less than 20 cm
    { 
      initRight  = true;
      if (initForward)
      {
        resetPIDVar();
      }
      initForward= false;

      // calculate the value for ahead/follow the wall
      pidValue = round(KP_FORWARD * error + KI_FORWARD * integral + KD_FORWARD * derivative);
      // matching the PID value to a minimum of -127 and a maximum of 127
      pidValue = fitCorrection(pidValue);
    }

    else 
    { // when the error is greater than or equal to 20 cm
      // Turn Right
      initForward= true;
      if (initRight)
      {
        resetPIDVar();
      }
      initRight = false;

      // calculate the PID value and turn left
      pidValue = round(KP_RIGHT * error + KI_RIGHT *integral +KD_RIGHT* derivative);
      // matching the PID value to a minimum of -127 and a maximum of 127
      pidValue = fitCorrection(pidValue);
      // the robot moves forward for a moment
      moveForward(128, 128);
      delay(70);
    }


    // write the left pwm(EN_L) and right pwm(EN_R) value with the value obtained by PID
    moveForward(128 + pidValue, 128 - pidValue);
  }
}