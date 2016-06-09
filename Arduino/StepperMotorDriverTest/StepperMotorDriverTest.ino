/*************************
Joel Bartlett
SparkFun Electronics
December 27, 2012

This code controls a stepper motor with the 
EasyDriver board. It spins forwards and backwards
***************************/
int dirpin = 11;
int steppin = 12;

void setup() 
{
Serial.begin(9600);
pinMode(dirpin, OUTPUT);
pinMode(steppin, OUTPUT);
}
void loop()
{

  int i;

  digitalWrite(dirpin, LOW);     // Set the direction.
  delay(2000);

  Serial.println("Moving Forward");
  for (i = 0; i<1000; i++)       // Iterate for 4000 microsteps.
  {
    digitalWrite(steppin, LOW);  // This LOW to HIGH change is what creates the
    digitalWrite(steppin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
    delayMicroseconds(500);      // This delay time is close to top speed for this
  }                              // particular motor. Any faster the motor stalls.

  digitalWrite(dirpin, HIGH);    // Change direction.
  delay(2000);

  Serial.println("Moving Backward");
  for (i = 0; i<1000; i++)       // Iterate for 4000 microsteps
  {
    digitalWrite(steppin, LOW);  // This LOW to HIGH change is what creates the
    digitalWrite(steppin, HIGH); // "Rising Edge" so the easydriver knows to when to step.
    delayMicroseconds(500);      // This delay time is close to top speed for this
  }                              // particular motor. Any faster the motor stalls.

}
