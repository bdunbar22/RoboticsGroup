//Test button

#include <Servo.h>

Servo servoLeft;          // Define left servo
Servo servoRight;         // Define right servo

int switch1 = 2; // connect a push button switch between this pin and ground
int ledpin = 13; // internal led, external LED, relay, trigger for other function, some other device, whatever.
boolean flag = false;
boolean servo_enable = false;

void setup()
{
  pinMode(ledpin,OUTPUT); // this pin controlled by flipflop() function
  pinMode (switch1,INPUT_PULLUP); // keeps pin HIGH via internal pullup resistor unless brought LOW with switch
  Serial.begin(9600); // just for debugging, not needed.
  servoLeft.attach(10);  // Set left servo to digital pin 10
  servoRight.attach(9);  // Set right servo to digital pin 9
  testInputs();
  stopRobot();
}

void loop()
{ 
  if (digitalRead(switch1)==LOW){
    delay(5); // I don't REALLY know why this delay helps, but it does. 
    flipflop(); // hops out of main loop and runs the flipflop function
  }

  if (servo_enable){
    forward();
    delay(1000);
    reverse();
    delay(1000);
    turnRight();
    delay(1000);
    stopRobot();         
    turnLeft();
    delay(1000);         
    stopRobot();
  }
  else{
    stopRobot();
  }

} // end of main loop.

void flipflop(){  //funtion flipflop 
  flag = !flag;  // since we are here, the switch was pressed So FLIP the boolian "flag" state (we don't even care if switch was released yet)
  Serial.print("flag =   " );   Serial.println(flag);   // not needed, but may help to see what's happening.

  if (flag){
    digitalWrite(ledpin,HIGH ); // if the flag var is HIGH turn the pin on
    servo_enable = true;
  }
  else {
    servo_enable = false;
    digitalWrite(ledpin,LOW); // if the flag var is LOW turn the pin off 
  }
  
  while(digitalRead(switch1)==LOW); // for "slow" button release, keeps us in the function until button is UN-pressed
  // If you take out this "while" the function becomes a flipflop oscillator if the button is held down. 
  delay(50); // OPTIONAL - play with this value.  It is probably short enough to not cause problems. deals with very quick switch press.
}

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
  servoLeft.write(95);
  servoRight.write(97); //debugged for inconsistent motors                
}


void testInputs() {
  for(int i = 85; i < 100; i+= 1) {
    servoLeft.write(i);
    servoRight.write(i);
    Serial.print("value =   " );   Serial.println(i);
    delay(2000);
  }
}

