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


/* ==============================================================================================================================================
 * Setup
 */
void setup()
{
  pinMode(ledpin,OUTPUT);                         // this pin controlled by flipflop() function
  pinMode (switch1,INPUT_PULLUP);                 // keeps pin HIGH via internal pullup resistor unless brought LOW with switch
  Serial.begin(9600);                             // just for debugging, not needed.
//  testInputs();
//  stopRobot();
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

  //Serial.println("Check enable");
  if (servo_enable){
    rightDistance = ultrasonicRight.Ranging(CM);
    forwardDistance = ultrasonicForward.Ranging(CM);

    Serial.print("Foward distance: "); Serial.println(forwardDistance);
    Serial.print("Right distance: "); Serial.println(rightDistance);


    if(forwardDistance < 12) {
      Serial.println("Rotate left.");
      rotateLeft(90);
    } else {
      if(rightDistance > 25 && rightDistance < 60) {
        Serial.println("turn right.");
        turnRight();
        forward(100);
      } else if(rightDistance < 10) {
        Serial.println("turn left sharp.");
        rotateLeft(12);
        turnLeft();
        forward(100);
      } else if(rightDistance < 20) {
        Serial.println("turn left.");
        turnLeft();
        forward(100);
      } else if (rightDistance > 60 && lastRightDistance < 60) {
        Serial.println("Rotate right.");
        forward(1200);
        rotateRight(90);
        forward(200);
      } else {
        /* Smooth out tragectory while in desired zone. */
        //if(rightDistance - lastRightDistance > 3) {
        //  turnRight();
        //  forward(100);
        //} else if(lastRightDistance - rightDistance > 3) {
        //  turnLeft();
        //  forward(100);
        //}
        Serial.println("Advance.");
        forward(50);
      }
    }

    lastRightDistance = rightDistance;
    
    //Serial.println("Running!");
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

/*
void testInputs() {
  for(int i = 85; i < 100; i+= 1) {
    servoLeft.write(i);
    servoRight.write(i);
    Serial.print("value =   " );   Serial.println(i);
    delay(2000);
  }
}
*/
