/*
RX is digital pin 2 (connect to TX of other device)
TX is digital pin 3 (connect to RX of other device)
*/

#include <SoftwareSerial.h>
SoftwareSerial mySerial(12, 11); // RX, TX

void setup()
{
  Serial.begin(9600);
  while (!Serial) {
  }

  Serial.println("Goodnight moon!");

  mySerial.begin(9600);
  mySerial.println("Hello, world?");
}

void loop()
{
  if (mySerial.available())
    Serial.write(mySerial.read());
  if (Serial.available())
    mySerial.write(Serial.read());
}
