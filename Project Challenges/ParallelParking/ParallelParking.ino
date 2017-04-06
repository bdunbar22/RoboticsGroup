




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
const int BRAKE_LIGHT_PIN = 2;
const float ANGLE_TO_TIME_MULTIPLIER = 7.8;       //degrees * milliseconds/degrees = milliseconds to run 
const float DISTANCE_THRESHHOLD = 20;               //20cm
 
Servo servoLeft;                                  // Define left servo
Servo servoRight;                                 // Define right servo

Ultrasonic ultrasonicFront(10, 11);                 // Trig then Echo pins
Ultrasonic ultrasonicRightFront(8, 9);
Ultrasonic ultrasonicRightBack(6, 7); 
Ultrasonic ultrasonicBack(4, 5);            

boolean flag = false;
boolean servo_enable = false;

long front;
long rightFront;
long rightBack;
long back;

int distanceCounter = 0; 
int lengthCounter = 30; //idk come up with the length
float distBuff = 1; // distance buffer

/* ==============================================================================================================================================
 * Setup
 * Setup will also run the full Parallel Parking procedure as we don't want to continuously park in new spots over time.
 */
void setup()
{
  //SET PINS
  pinMode (BUZZER_PIN, OUTPUT);
  pinMode (BRAKE_LIGHT_PIN, OUTPUT);
  beep();
  Serial.begin(9600);                             // just for debugging, not needed.
}

/* ==============================================================================================================================================
 * Loop
 */
void loop()
{
  /* 
  if (digitalRead(switch1)==HIGH){
    delay(5); 
    flipflop(); 
  }
*/
//  if (servo_enable){
//    rightDistance1 = ultrasonicRight1.Ranging(CM);
//    rightDistance2 = ultrasonicRight2.Ranging(CM);
//    forwardDistance = ultrasonicFront.Ranging(CM);
//    backDistance = ultrasonicBack.Ranging(CM);
    updateDistances();
    Serial.print("Front: "); Serial.print(front);
    Serial.print("    rightFront: "); Serial.print(rightFront);
    Serial.print("    rightBack: "); Serial.print(rightBack);
    Serial.print("    Back: "); Serial.println(back);


    //forward(50);
    //Serial.println("Move Foward.");
    delay(100);
    /*
     * Let's do 4 sensors!
     * 1 forward
     * 2 on the right
     * 1 back
     * 
     * Having 2 on the right allows us to make sure we are parrallel to the wall at first. 
     * Step 1, approach wall until a desired distance.
     * Step 2, use both right sensors to rotate until parrallel.
     * Step 3, move forward past the wall
     * Step 4, re-ensure correct distance
     * Step 5, park procedure
     * Step 6, correct park job to be close to inside wall.
     */


 //move forward
 /*
 if rightDistance > distanceThreshold{
   distanceCounter++
   wallDistance = rightDistance;
   }
  if (distanceCounter < lengthCounter && rightDistance < wallDistance + distBuff){
    distanceCounter = 0;    
  }
  if distanceCounter > lengthCounter{
   //begin parallel park

     if (right distance <= wallDistance + distBuff){
       forward(500); //length of car
       reverse(500);
       turnRight(); //45 degrees
       reverse(500);
       turnLeft();// 45 degrees
     }
   }

    lastRightDistance = rightDistance;
    
  }
  else{
    detachRobot();
  }
  */

  delay(3); // Try not to draw too much power
  //Serial.println("End of loop");
}                                                 // end of main loop.

/* ==============================================================================================================================================
 * Functions
 */

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


void beep() {
  digitalWrite(BRAKE_LIGHT_PIN, HIGH);
  tone(BUZZER_PIN, 100, 300);  
  delay(600);
  tone(BUZZER_PIN, 100, 700); 
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
}

void forward(int moveTime) {
  attachRobot();
  servoLeft.write(180);
  servoRight.write(0);
  delay(moveTime);
  detachRobot();
}

void reverse(int moveTime) {
  attachRobot();
  servoLeft.write(0);
  servoRight.write(180);
  delay(moveTime);
  detachRobot();
}

/**
 * While moving forward, turn a few degrees to the right
 * debugged for a small angle
 */
void turnRight() {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(180);
  delay(5*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * Rotate in place
 */
void turnLeft() {
  attachRobot();
  servoLeft.write(0);
  servoRight.write(93);
  delay(5*ANGLE_TO_TIME_MULTIPLIER);
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

void detachRobot() {
  servoLeft.detach();
  servoRight.detach();
}

void attachRobot(){
  servoLeft.attach(SERVO_LEFT_PIN);  
  servoRight.attach(SERVO_RIGHT_PIN); 
}


