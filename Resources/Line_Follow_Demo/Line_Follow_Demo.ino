/* ==============================================================================================================================================
 * Ben Dunbar, Gareth Moo, Eric Shi
 * 
 * Robotics Lab
 * Line Following Robot
 * 3/17/17
 * 
 * This robot will follow a black line on a piece of wood or other light background.
 * When the left sensor no longer recieves reflected light, we will assume it is over the black line and turn left to correct our direction.
 * When the right sensor no longer recieves reflected light, we will turn right.
 * If both sensors are recieving light normally, we will continue forward.
 * 
 * We have included code to stop the robot by putting a hand right in front of a sonar sensor on the robot.
 * 
 * To start the robot, press the red button on the front of the robot.
 */

/* ==============================================================================================================================================
 * Includes 
 */
#include <Servo.h>

/* ==============================================================================================================================================
 * Declare variables
 */
Servo servoLeft;                                  // Define left servo
Servo servoRight;                                 // Define right servo

int switch1 = 2;                                  // connect a push button switch between this pin and ground
int ledpin = 13;                                  // internal led, external LED, relay, trigger for other function, some other device, whatever.
boolean flag = false;
boolean servo_enable = true;
float ANGLE_TO_TIME_MULTIPLIER = 7.8;                      //degrees * milliseconds/degrees = milliseconds to run 
int memory = 0;                       

#define LEFT_IR A0
#define CENTER_IR A1
#define RIGHT_IR A2
#define LEFT_VAL 105
#define CENTER_VAL 105
#define RIGHT_VAL 115


long forwardDistance;


/* ==============================================================================================================================================
 * Setup
 */
void setup()
{
  pinMode(ledpin,OUTPUT);                         // this pin controlled by flipflop() function
  pinMode (switch1,INPUT_PULLUP);                 // keeps pin HIGH via internal pullup resistor unless brought LOW with switch
  Serial.begin(9600);                             // just for debugging, not needed.
//  forward(100);
//
//  rotateRight(90);
//  
//  rotateLeft(90);
//
//forward(15);
//
//  delay(500);
//
//  turnRight();
//
//  delay(500);
//
//  turnLeft();
}

/* ==============================================================================================================================================
 * Loop
 */
void loop()
{ 
  //memory -1 = left
  //        1 = right
  Serial.print("L : "); Serial.print(analogRead(A0)); Serial.print(" C: "); Serial.print(analogRead(A1));
  Serial.print(" R : "); Serial.println(analogRead(A2));
  if(servo_enable) {
    // Going through all combinations via binary counting
    // choose action based on sensors
    // 2^3 possible sensor combinations
    if(!leftBlack() && !centerBlack() && !rightBlack()) {
        Serial.print("____  ____  ____\n");
        if (memory == 1) {
          rotateRight(20);
          memory = 0;
        } else if(memory == -1) {
          rotateLeft(20);
          memory = 0;
        } 
//        forward(25);
    }
    else if(!leftBlack() && !centerBlack() && rightBlack()) {
      Serial.print("____  ____  RIGHT\n");
      rotateRight(45);
      forward(35);
      memory = 1;
    }
      else if(!leftBlack() && centerBlack() && !rightBlack()) {
      Serial.print("____  CENTER  ____\n");
      forward(25);
      memory = 0;
    }
    else if(!leftBlack() && centerBlack() && rightBlack()) {
      Serial.print("____  CENTER  RIGHT\n");
      forward(5);
      if(!(leftBlack() && centerBlack() && rightBlack())) {
        while(!(!leftBlack() && centerBlack() && !rightBlack()) || (leftBlack() && centerBlack() && !rightBlack())) {
          turnRight();
          forward(20);
        }
      }
      memory = 1;
    }
    else if(leftBlack() && !centerBlack() && !rightBlack()) {
      Serial.print("LEFT  ____  ____\n");
      rotateLeft(45);
      forward(35);
      memory = -1;
    }
    else if(leftBlack() && !centerBlack() && rightBlack()) {
      Serial.print("LEFT  ____  RIGHT\n");
      forward(25);
    }
    else if(leftBlack() && centerBlack() && !rightBlack()) {
      Serial.print("LEFT  CENTER  ____\n");
      forward(5);
      if(!(leftBlack() && centerBlack() && rightBlack())) {
        while(!(!leftBlack() && centerBlack() && !rightBlack()) || (!leftBlack() && centerBlack() && rightBlack())) {
          turnLeft();
          forward(20);
        }
      }
      memory = -1;
    }
    else if(leftBlack() && centerBlack() && rightBlack()) {
      Serial.print("LEFT CENTER RIGHT\n");
      forward(25);
//      if (memory == 1) {
//        rotateRight(20);
//      } else if(memory == -1) {
//        rotateLeft(20);
//      } else {
        if(leftBlack() && centerBlack() && rightBlack()) {
           //If still black stop
            detachRobot();
            servo_enable = false;
        } 
      }
  }
  delay(1); // Try not to draw too much power or go to fast.
}                                                 

/* ==============================================================================================================================================
 * Functions
 */

boolean leftBlack() {
  return analogRead(LEFT_IR) < LEFT_VAL;
}

boolean centerBlack() {
  return analogRead(CENTER_IR) < CENTER_VAL;
}

boolean rightBlack() {
  return analogRead(RIGHT_IR) < RIGHT_VAL;
}

 
void flipflop(){                                        //funtion flipflop 
  flag = !flag;                                         // since we are here, the switch was pressed So FLIP the boolian "flag" state 
                                                        //    (we don't even care if switch was released yet)
//  Serial.print("flag =   " );   Serial.println(flag);   // not needed, but may help to see what's happening.

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
  servoLeft.write(180);
  servoRight.write(93);
  delay(5*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * Rotate in place
 */
void turnLeft() {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(0);
  delay(5*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * Rotate in place
 */
void rotateRight(int degree) {
  attachRobot();
  servoLeft.write(180);
  servoRight.write(93);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

/**
 * Rotate in place
 */
void rotateLeft(int degree) {
  attachRobot();
  servoLeft.write(93);
  servoRight.write(0);
  delay(degree*ANGLE_TO_TIME_MULTIPLIER);
  detachRobot();
}

void detachRobot() {
  servoLeft.detach();
  servoRight.detach();
}

void attachRobot(){
  servoLeft.attach(9);  
  servoRight.attach(10); 
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
