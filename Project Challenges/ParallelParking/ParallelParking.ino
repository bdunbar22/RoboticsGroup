/* ==============================================================================================================================================
 * Ben Dunbar, Gareth Moo, Eric Shi
 * 
 * Robotics Lab
 * Parallel Parking Robot
 * 4/5/17
 * 
 * Robot Design:
 * We have devized robot algorithm which controls a robot with two continuous servos controlling a left and right wheel for motion. 
 * The robot uses four HC-SR04 sonar sensors to get data to model the world around it; they are located: front, two on the right side, back.
 * The chassis we have used in our design is the boe-bot by parallax.
 * We have made the program on an Arduino Nano. 
 * The servos have a separate 6V power source from the nano, but the control signals are sent by a PWM signal from the Nano on pins d2 and d3.
 * The sonar sensors have been given digital pins 5-12.
 * Sonar signals are in CM for our approach.
 * 
 * Algorithm:
 * By using the two right sensors together, we can get a good idea of the angle between the robot and a wall on the right side of the robot.
 * This can be used to give us a greater degree of control on our robot so that we can align it parallel to the wall at a desired distance and 
 * then proceed with a fairly fixed procedure.
 */

/* ==============================================================================================================================================
 * Includes 
 */
#include <Servo.h>
#include <Ultrasonic.h>


/* ==============================================================================================================================================
 * Declare variables
 */
#define CM 1

const int SERVO_RIGHT_PIN = 12;
const int SERVO_LEFT_PIN = 13; 
const int BUZZER_PIN = 3;
const int REVERSE_LIGHT_PIN = 2;
const int PUSH_BUTTON = 14;

const float ANGLE_TO_TIME_MULTIPLIER = 7.8;       //degrees * milliseconds/degrees = milliseconds to run 
const float DISTANCE_THRESHHOLD = 20;             //20cm
 
Servo servoLeft;                                  // Define left servo
Servo servoRight;                                 // Define right servo

Ultrasonic ultrasonicFront(10, 11);               // Trig then Echo pins
Ultrasonic ultrasonicRightFront(8, 9);
Ultrasonic ultrasonicRightBack(6, 7); 
Ultrasonic ultrasonicBack(4, 5);            

long front;
long rightFront;
long rightBack;
long back;


/* ==============================================================================================================================================
 * Setup
 * Setup will also run the full Parallel Parking procedure as we don't want to continuously park in new spots over time.
 */
void setup()
{
  // SET PINS
  pinMode (BUZZER_PIN, OUTPUT);
  pinMode (REVERSE_LIGHT_PIN, OUTPUT);
  pinMode (PUSH_BUTTON, INPUT);
  
  // Start Serial for debugging
  Serial.begin(9600);        
  int button = digitalRead(PUSH_BUTTON);
  Serial.println(button);
  // Start when push button is pushed
  while(button == LOW) {
     button = digitalRead(PUSH_BUTTON);
//     Serial.println(button);
     reverseLightOn();
     delay(100);
     reverseLightOff();
     delay(100);
  }
  // Run parallel park procedures.
  parallelPark();                     
}

void parallelPark() 
{
  beep();

  // Step 
  // Go forward to first box
  forward();
  updateDistances();
  while(rightFront > 20 || rightBack > 20) {
    updateDistances();
  }
  stopRobot();

  // Step 
  // align
  while(rightFront != rightBack) {
    int align = rightFront - rightBack;
    if(align > 0) {
      //rotateRight
    } else if(align < 0) {
      //rotateLeft
    }
  }

  
  
  // Step 
  // Go all the way past gap to next box

  // Step 
  // Re align

  // Step 
  // Back up slightly
  reverseLightOn();

  // Step  
  // Turn and go into spot

  // Step 
  // Adjust in spot

  // Step 
  // Leave spot
  reverseLightOff();

  // Step  
  // Leave course
}

/* ==============================================================================================================================================
 * Loop
 */
void loop()
{
    // No movement. Can still test out distances    
    updateDistances();
    printData();

    delay(50);
}                                                 

/* ==============================================================================================================================================
 * Functions
 */
/* ========================================== DATA ==========================================*/
void updateDistances() {
    fetchDistances();

    //Update again on error
    while(isBadDistance()) {
      fetchDistances();
    }
}

void fetchDistances() {
    rightFront = ultrasonicRightFront.Ranging(CM);
    rightBack = ultrasonicRightBack.Ranging(CM);
    front = ultrasonicFront.Ranging(CM);
    back = ultrasonicBack.Ranging(CM);
}

bool isBadDistance() {
  return (rightFront > 3000 || rightBack > 3000 || front > 3000 || back > 3000);
}

void printData() {
  Serial.print("Front: "); Serial.print(front);
  Serial.print("    rightFront: "); Serial.print(rightFront);
  Serial.print("    rightBack: "); Serial.print(rightBack);
  Serial.print("    Back: "); Serial.println(back);
}

/* ========================================== NOISE ==========================================*/
void beep() {
  tone(BUZZER_PIN, 100, 300);  
  delay(600);
  tone(BUZZER_PIN, 100, 700); 
  }

/* ========================================== LIGHTS ==========================================*/
void reverseLightOn() {
  digitalWrite(REVERSE_LIGHT_PIN, HIGH);
}

void reverseLightOff() {
  digitalWrite(REVERSE_LIGHT_PIN, LOW);
}

/* ========================================== MOTION ==========================================*/
void forward() {
  attachRobot();
  servoLeft.write(180);
  servoRight.write(0);
}

void reverse() {
  attachRobot();
  servoLeft.write(0);
  servoRight.write(180);
}

/**
 * While moving forward, turn a few degrees to the right
 * debugged for a small angle
 */
void turnRight(int degree) {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(180);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * While moving forward, turn a few degrees to the right
 * debugged for a small angle
 */
void turnLeft(int degree) {
  attachRobot();
  servoLeft.write(0);
  servoRight.write(93);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * Rotate in place
 */
void rotateRight(int degree) {
  attachRobot();
  servoLeft.write(180);
  servoRight.write(180);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * Rotate in place
 */
void rotateLeft(int degree) {
  attachRobot();
  servoLeft.write(0);
  servoRight.write(0);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

void stopRobot() {
  detachRobot();
}


/* ========================================== MOTOR CONFIG ==========================================*/
void detachRobot() {
  servoLeft.detach();
  servoRight.detach();
}

void attachRobot(){
  servoLeft.attach(SERVO_LEFT_PIN);  
  servoRight.attach(SERVO_RIGHT_PIN); 
}


