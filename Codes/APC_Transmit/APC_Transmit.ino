#include <SoftwareSerial.h>

SoftwareSerial mySerial(4, 5);  //(Rx, Tx

unsigned long time_now;
unsigned long time_stop = 20000;

void setup() {
  Serial.begin(38400);
  mySerial.begin(38400);

}

void loop()
{
 time_now = micros();
 Serial.println(10);
 mySerial.println(10);
 Serial.println(10000);
 mySerial.println(10000);
 while(micros()<= time_now+time_stop);
}
