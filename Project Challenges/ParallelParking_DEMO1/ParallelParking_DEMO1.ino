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
const float DISTANCE_THRESHHOLD = 21;             //25cm
const float PROXIMITY_THRESHHOLD = 10;
 
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
  
  // Run parallel park procedures.
  start();
  //approachWallBack();
  //park();
  parallelPark();                     
}

void parallelPark() 
{
  start();
  
  beep();

  // Step 1 
  // Go forward to first box
  approachBox();

  // Step 
  // align
  align();

  // Step
  // Don't allow too close to next wall
  if((rightFront + rightBack) / 2 < PROXIMITY_THRESHHOLD) {
    Serial.println("Adjust for proximity");
    rotateLeft(6); 
  }
  
  // Step 
  // Go all the way past gap to next box
  approachGap();
  approachBox();

  // Step 
  // Re align
  align();


  // Step
  // Perform Park
  park();

  
  // Step 
  // Leave spot
  unPark();

  // Step  
  // Leave course
  approachGap();
  forward();
  delay(500);
  stopRobot();
  beep();
}

/* ==============================================================================================================================================
 * Complex Robot Movement
 */
void start() {
  // Wait for button to start
  int button = digitalRead(PUSH_BUTTON);
  Serial.println(button);

  while(button == LOW) {
     button = digitalRead(PUSH_BUTTON);
     Serial.println(button);
     reverseLightOn();
     delay(100);
     reverseLightOff();
     delay(100);
     updateDistances();
     printDistances();
  }
  delay(25);
}

void approachBox() {
  forward();
  for(int i = 0; i < 3; i++) {
    Serial.print("Start Approach Box: "); Serial.println(i);
    updateDistances();
    printDistances();
    while(rightFront > DISTANCE_THRESHHOLD || rightBack > DISTANCE_THRESHHOLD) {
      Serial.println("Approach Box 1");
      updateDistances();
      printDistances();
      delay(5);
    }
  }
  delay(475);
  stopRobot();
}

void approachGap() {
  forward();
  updateDistances();
  while(rightFront < DISTANCE_THRESHHOLD || rightBack < DISTANCE_THRESHHOLD) {
    updateDistances();
    Serial.println("Approach Gap");
    printDistances();
    delay(5);
  }
}
 
void align() {
  delay(100);
  updateDistances();
  int count = 0;
  while(!(rightFront == rightBack || rightFront - rightBack == 1)) {
    count++;
    if(count > 2) {
      forward();
      delay(20);
      stopRobot();
    }
    Serial.println("Aligning 1");
    int align = rightFront - rightBack;
    if(align > 0) {
      rotateRight(4);
    } else if(align < 0) {
      rotateLeft(4);
    }
    delay(100);
    updateDistances();
    printDistances();
    Serial.println("Aligning 2");
  }
}

void park() {
  align();
  while((rightFront + rightBack) / 2 > 10) {
    approachWallBack();
  }
  // Step 
  // Back up slightly
  reverseLightOn();
  reverse();
  updateDistances();
  while(rightBack < DISTANCE_THRESHHOLD) {
    updateDistances();
    delay(5);
  }
  delay(425);
  stopRobot();
  
  // Step  
  // Turn
  turnLeftReverse(75);
  
  // Step  
  // go into spot
  reverse();
  updateDistances();
  while(back > 20) {
    updateDistances();
    delay(5);
  }
  delay(150);
  Serial.println("END OF PARK REVERSE");
  updateDistances();
  printDistances();
  stopRobot();

  // Step  
  // Correction Turn
  turnRightReverse(75);

  // Step
  // Fix park job
  align();
  beep();
  reverseLightOff();
}

void unPark() {
  turnLeft(100);

  forward();
  delay(700);
  stopRobot();
  
  turnRight(100);
  
  forward();
  delay(400);
  stopRobot();

  turnRight(32);
  
  forward();
  delay(200);
  stopRobot();
  
  align();
}

void approachWallBack() {
  reverseLightOn();
  turnLeftReverse(60);
  reverse();
  delay(20);
  turnRightReverse(80);
  delay(50);
  reverseLightOff();
  forward();
  delay(350);
  stopRobot();
  align();
  Serial.println("Approached wall");
  updateDistances();
  printDistances();
}


/* ==============================================================================================================================================
 * Loop
 */
void loop()
{
  updateDistances();
  printDistances();
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

void printDistances() {
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
  delay(400);
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
  servoLeft.write(180);
  servoRight.write(93);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * While moving forward, turn a few degrees to the left
 * debugged for a small angle
 */
void turnLeft(int degree) {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(0);
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


/**
 * While moving reverse, turn a few degrees to the right
 * debugged for a small angle
 */
void turnRightReverse(int degree) {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(180);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * While moving reverse, turn a few degrees to the left
 * debugged for a small angle
 */
void turnLeftReverse(int degree) {
  attachRobot();
  servoLeft.write(0);
  servoRight.write(93);
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


