//LIDAR
//GND ->Power GND
//V5.0 -> 5V supply for core
//VMOTO -> 5V supply for motor
//RX -> Pin1 (TXD)
//TX -> Pin0 (RXD)
//MOTOCTL -> Pin3 (PWM)

//Using a lot of the code from the simple_connect function 
#include <RPLidar.h>
RPLidar lidar;

#define RPLIDAR_MOTOR 3 // The PWM pin for control the speed of RPLIDAR's motor.
                        // This pin should connected with the RPLIDAR's MOTOCTRL signal
                        //Can also connect MOTOCTRL to the Arduino 3.3V pin for maximum rotational speed 
int forward_angle = 0;
int right_angle = 90;
int Left_angle = 270;
float distance_forward;
float distance_right;
float distance_left;


bool transmit = false;
byte incomingByte;
//Distance is read in mm
int maxDistance = 6000; // 6 meters
int minDistance = 1000; // 1 meter

//Servo
#include <Servo.h>

Servo servoLeft;          // Define left servo
Servo servoRight;         // Define right servo
void setup() {
  Serial.begin(9600);   //Sets the baud for serial data transmission                               

  servoLeft.attach(6);  // Set left servo to digital pin 10
  servoRight.attach(5);  // Set right servo to digital pin 9
  // LIDAR pins
  pinMode(LED_BUILTIN, OUTPUT); // For demo
}

void loop() {

  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance; //distance value in mm unit
    float angle    = lidar.getCurrentPoint().angle; //anglue value in degree
    bool  startBit = lidar.getCurrentPoint().startBit; //whether this point is belong to a new scan
    byte  quality  = lidar.getCurrentPoint().quality; //quality of the current measurement
    int haptic_motor[8];  // output of haptic motors
    
    if(Serial.available() > 0) {
      incomingByte = Serial.read();
      transmit = !transmit;
    }
    //set distance values for different dirrections
    if(angle > forward_angle && angle <= forward_angle+0.5 && transmit) {
      distance_forward = distance;
      }
    if(angle > right_angle && angle <= right_angle+0.5 && transmit) {
      distance_right = distance;
      }
    if(angle > left_angle && angle <= left_angle+0.5 && transmit) {
      distance_left = distance;
      }

//Actual moving

      if (distance_forward > 50 && /*enable_b*/) {
        forward();
       } else {
          
         stopRobot();
          if(distance_right > distance_left){
            turnright();
            delay(200);
            stopRobot();
          }
          else if(distance_left > distance_right){
            turnleft();
            delay(200);
            stopRobot();
           } 
        }

  }
}

// Motion routines for forward, reverse, turns, and stop
void forward() {
  servoLeft.write(0);
  servoRight.write(180);
}
void reverse() {
  servoLeft.write(180);
  servoRight.write(0);
}
void turnRight() {
  servoLeft.write(180);
  servoRight.write(180);
}
void turnLeft() {
  servoLeft.write(0);
  servoRight.write(0);
}
void stopRobot() {
  servoLeft.write(90);
  servoRight.write(90);
  servoLeft.detach(); 
  servoRight.detach(); 
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(50);                       
  digitalWrite(LED_BUILTIN, LOW);
  servoRight.attach(9);
  servoLeft.attach(10);
}

