#include "Ultrasonic.h"
Ultrasonic ultrasonic( 12, 13 );

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
