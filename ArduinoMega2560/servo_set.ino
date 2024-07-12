#include <Servo.h>

Servo myservo;  // create servo object to control a servo

void setup() {
  Serial.begin(9600);       // initialize serial communication at 9600 baud rate
  myservo.attach(6);        // attaches the servo on pin 9 to the servo object
}

void loop() {
  // Set servo to 0 degrees
  myservo.write(90);
  Serial.println("Position: 90 degrees");
  delay(1000);
  
  myservo.write(0);
  Serial.println("Position: 90 degrees");
  delay(1000);
  myservo.write(180);
  Serial.println("Position: 90 degrees");
  delay(1000);
  
}
