
/* ==============================================================================================================================================
 * Ben Dunbar, Gareth Moo, Eric Shi
 * 
 * Robotics Lab
 * Wall Following Robot
 * 3/17/17
 * 
 */

/* ==============================================================================================================================================
 * Includes 
 */
#include <Servo.h>
#include "Ultrasonic.h"

/* ==============================================================================================================================================
 * Declare variables
 */
Servo servoLeft;                                  // Define left servo
Servo servoRight;                                 // Define right servo
Ultrasonic ultrasonicRight( 8, 11 );              // Trig then Echo pins
Ultrasonic ultrasonicForward( 4, 5 );           

int switch1 = 2;                                  // connect a push button switch between this pin and ground
int ledpin = 13;                                  // internal led, external LED, relay, trigger for other function, some other device, whatever.
boolean flag = false;
boolean servo_enable = false;
float ANGLE_TO_TIME_MULTIPLIER = 7.8;                      //degrees * milliseconds/degrees = milliseconds to run 

#define CM 1

long forwardDistance;
long rightDistance;
long lastRightDistance = 150;

float distanceThreshold = 20; //20cm
int distanceCounter = 0; 
int lengthCounter = 30; //idk come up with the length
float distBuff = 1; // distance buffer

/* ==============================================================================================================================================
 * Setup
 */
void setup()
{
  pinMode(ledpin,OUTPUT);                         // this pin controlled by flipflop() function
  pinMode (switch1,INPUT_PULLUP);                 // keeps pin HIGH via internal pullup resistor unless brought LOW with switch
  Serial.begin(9600);                             // just for debugging, not needed.
}

/* ==============================================================================================================================================
 * Loop
 */
void loop()
{ 
  if (digitalRead(switch1)==HIGH){
    delay(5); 
    flipflop(); 
  }

  if (servo_enable){
    rightDistance = ultrasonicRight.Ranging(CM);
    forwardDistance = ultrasonicForward.Ranging(CM);
    Serial.print("Foward distance: "); Serial.println(forwardDistance);
    Serial.print("Right distance: "); Serial.println(rightDistance);


 //move forward
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

  delay(3); // Try not to draw too much power
  //Serial.println("End of loop");
}                                                 // end of main loop.

/* ==============================================================================================================================================
 * Functions
 */
void flipflop(){                                        //funtion flipflop 
  flag = !flag;                                         // since we are here, the switch was pressed So FLIP the boolian "flag" state 
                                                        //    (we don't even care if switch was released yet)
  Serial.print("flag =   " );   Serial.println(flag);   // not needed, but may help to see what's happening.

  if (flag){
    digitalWrite(ledpin,HIGH );                         // if the flag var is HIGH turn the pin on
    servo_enable = true;
  }
  else {
    servo_enable = false;
    digitalWrite(ledpin,LOW);                           // if the flag var is LOW turn the pin off 
  }
  
  while(digitalRead(switch1)==HIGH);                     // for "slow" button release, keeps us in the function until button is UN-pressed
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
  servoLeft.attach(10);  
  servoRight.attach(9); 
}
