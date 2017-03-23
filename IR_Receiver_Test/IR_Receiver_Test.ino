#include <IRremote.h>

int RECV_PIN = 11;

IRrecv irrecv(RECV_PIN);
//IRsend irsend;

decode_results results;

void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  tone(8, 38000);
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.println("Recieving data");
    irrecv.resume(); // Receive the next value
  } else {
    Serial.println("Didn't receive data.");
  }
  delay(25);
}
