#include "Ultrasonic.h"
Ultrasonic ultrasonic( 8, 11 );

void setup()
{
 Serial.begin( 115200 );
}

void loop()
{
 Serial.print( ultrasonic.Ranging(CM) );
 Serial.println( "cm" );
   
 delay(10);
}
