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
Ultrasonic ultrasonicForward( 8, 11 );            // Trig then Echo pins
Ultrasonic ultrasonicRight( 12, 13 );           

int switch1 = 2;                                  // connect a push button switch between this pin and ground
int ledpin = 13;                                  // internal led, external LED, relay, trigger for other function, some other device, whatever.
boolean flag = false;
boolean servo_enable = false;
float ANGLE_TO_TIME_MULTIPLIER = 6.055555;                      //seconds/degrees

#define CM 1

long forwardDistance;
long rightDistance;


/* ==============================================================================================================================================
 * Setup
 */
void setup()
{
  pinMode(ledpin,OUTPUT);                         // this pin controlled by flipflop() function
  pinMode (switch1,INPUT_PULLUP);                 // keeps pin HIGH via internal pullup resistor unless brought LOW with switch
  Serial.begin(9600);                             // just for debugging, not needed.
  servoLeft.attach(10);                           // Set left servo to digital pin 10
  servoRight.attach(9);                           // Set right servo to digital pin 9
//  testInputs();
//  stopRobot();
}

/* ==============================================================================================================================================
 * Loop
 */
void loop()
{ 

  forwardDistance = ultrasonicForward.Ranging(CM);
  rightDistance = ultrasonicRight.Ranging(CM);

  Serial.print("Foward distance: "); Serial.print(ultrasonicForward.Ranging(CM));
  Serial.print(" Right distance: "); Serial.println(rightDistance);
  
  if (digitalRead(switch1)==LOW){
    delay(5); 
    flipflop(); 
  }

  if (servo_enable){
    attachRobot();

    Serial.println("Running!");
 
  }
  else{
    detachRobot();
  }
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
  
  while(digitalRead(switch1)==LOW);                     // for "slow" button release, keeps us in the function until button is UN-pressed
}

void forward() {
  servoLeft.write(180);
  servoRight.write(0);
}

void reverse() {
  servoLeft.write(0);
  servoRight.write(180);
}

/**
 * While moving forward, turn a few degrees to the right
 * debugged for a small angle
 */
void turnRight() {
  servoLeft.write(93);
  servoRight.write(180);
  delay(5*ANGLE_TO_TIME_MULTIPLIER);
}

/**
 * Rotate in place
 */
void turnLeft() {
  servoLeft.write(0);
  servoRight.write(93);
  delay(5*ANGLE_TO_TIME_MULTIPLIER);
}

/**
 * Rotate in place
 */
void rotateRight(int degree) {
  servoLeft.write(180);
  servoRight.write(180);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
}

/**
 * Rotate in place
 */
void rotateLeft(int degree) {
  servoLeft.write(0);
  servoRight.write(0);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
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
