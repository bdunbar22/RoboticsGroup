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
float ANGLE_TO_TIME_MULTIPLIER = 5;                      //degrees * milliseconds/degrees = milliseconds to run 

#define CM 1

long forwardDistance;
long rightDistance;
long lastRightDistance = -1;
int lastTurn = 0;             // -1 turned left, 1 turned right. all others are not counted.


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

  rightDistance = ultrasonicRight.Ranging(CM);
  forwardDistance = ultrasonicForward.Ranging(CM);

  Serial.print("Foward distance: "); Serial.print(forwardDistance);
  Serial.print("  Right distance: "); Serial.print(rightDistance);
  Serial.print("  Last Distance: "); Serial.print(lastRightDistance);
  Serial.print("  Last turn: "); Serial.print(lastTurn);
  
  //Serial.println("Check enable");
  if (servo_enable){
   
    if(forwardDistance < 12) {
      forwardDistance = ultrasonicForward.Ranging(CM);
      if(forwardDistance < 12) {
        Serial.println("  rotate left.");
        rotateLeft(90);
        lastRightDistance = -1;
        lastTurn = 0;
      }
    } else {
      if (lastRightDistance != -1 && rightDistance > 70 && rightDistance - lastRightDistance > 30) {
        Serial.println("  check rotate right.");
        forward(100);
        
        rightDistance = ultrasonicRight.Ranging(CM);
       
        if (lastRightDistance != -1 && rightDistance > 70 && rightDistance - lastRightDistance > 30) {
          Serial.println("Rotate right.");
          forward(1100);
          rotateRight(90);
          forward(300);
          lastRightDistance = -1;
          lastTurn = 0;
        }
        
      } else {
        if((lastRightDistance != -1 && rightDistance > lastRightDistance) && lastTurn != -1) {
          Serial.println("  turn right.");
            turnRight();
            forward(100);
        } else if(rightDistance < 15 || (lastRightDistance != -1 && lastRightDistance > rightDistance) && lastTurn != 1) {
          Serial.println("  turn left.");
          turnLeft();
          forward(100);
        } else {
          Serial.println("  forward.");
          forward(100);
          lastTurn = 0;
        }
        lastRightDistance = rightDistance;
      }
    }
    
    //Serial.println("Running!");
  }
  else{
    detachRobot();
    Serial.println("");
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
  delay(15*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
  lastTurn = 1;
}

/**
 * Rotate in place
 */
void turnLeft() {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(0);
  delay(15*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
  lastTurn = -1;
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
