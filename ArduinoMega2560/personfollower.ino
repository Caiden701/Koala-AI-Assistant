/*
#include <Wire.h>

#include "person_sensor.h"

// How long to wait between reading the sensor. The sensor can be read as
// frequently as you like, but the results only change at about 5FPS, so
// waiting for 200ms is reasonable.
const int32_t SAMPLE_DELAY_MS = 200;

void setup() {
  // You need to make sure you call Wire.begin() in setup, or the I2C access
  // below will fail.
  Wire.begin();
  Serial.begin(9600);
}

void loop() {
  person_sensor_results_t results = {};
  // Perform a read action on the I2C address of the sensor to get the
  // current face information detected.
  if (!person_sensor_read(&results)) {
    Serial.println("No person sensor results found on the i2c bus");
    delay(SAMPLE_DELAY_MS);
    return;
  }

  Serial.println("********");
  Serial.print(results.num_faces);
  Serial.println(" faces found");
  for (int i = 0; i < results.num_faces; ++i) {
    const person_sensor_face_t* face = &results.faces[i];
    Serial.print("Face #");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(face->box_confidence);
    Serial.print(" confidence, (");
    Serial.print(face->box_left);
    Serial.print(", ");
    Serial.print(face->box_top);
    Serial.print("), (");
    Serial.print(face->box_right);
    Serial.print(", ");
    Serial.print(face->box_bottom);
    Serial.print("), ");
    if (face->is_facing) {
      Serial.println("facing");
    } else {
      Serial.println("not facing");
    }
  }
  delay(SAMPLE_DELAY_MS);
}
*/
#include <Wire.h>
#include <Servo.h>
#include "person_sensor.h"

// How long to wait between reading the sensor. The sensor can be read as
// frequently as you like, but the results only change at about 5FPS, so
// waiting for 200ms is reasonable.
const int32_t SAMPLE_DELAY_MS = 200;

// Create a Servo object
Servo myServoX;

// Define the pins for the servo motors
const int servoPinX = 6;

// Define the initial position of the servo motors
int servoPositionX = 90; // Center position


// Define the minimum and maximum positions for the servo motors
const int servoMinPosition = 0;
const int servoMaxPosition = 180;

const int faceDetectPin = 2;

void setup() {
  // Initialize I2C and Serial communication
  Wire.begin();
  Serial.begin(9600);

  // Attach the servos to the specified pins
  myServoX.attach(servoPinX);

  // Move the servos to the initial position
  myServoX.write(servoPositionX);

  // Initialize the face detection pin as output
  pinMode(faceDetectPin, OUTPUT);
  digitalWrite(faceDetectPin, LOW); // Initially set to LOW
}

void loop() {
  person_sensor_results_t results = {};
  
  // Perform a read action on the I2C address of the sensor to get the
  // current face information detected.
  if (!person_sensor_read(&results)) {
    Serial.println("No person sensor results found on the i2c bus");
    delay(SAMPLE_DELAY_MS);
    return;
  }

  Serial.println("********");
  Serial.print(results.num_faces);
  Serial.println(" faces found");
  
  if (results.num_faces > 0) {
    // Face(s) detected
    digitalWrite(faceDetectPin, HIGH); // Set pin 2 to HIGH
    const person_sensor_face_t* face = &results.faces[0]; // Track the first face detected
    
    int faceCenterX = (face->box_left + face->box_right)/2;

   
   

   //Calculate the target servo position based on the face coordinates
    int targetPositionX = map(faceCenterX, 0, 255, servoMaxPosition, servoMinPosition);
   
    // Constrain the target position to stay within the specified range
    targetPositionX = constrain(targetPositionX, servoMinPosition, servoMaxPosition);
   
    // Print the target position
    Serial.print("Target Position X: ");
    Serial.println(targetPositionX);
    // Move the servos to the target positions
    myServoX.write(targetPositionX);
    
    // Update the current servo positions
   //servoPositionX = targetPositionX;
  }else {
    // No face detected
    digitalWrite(faceDetectPin, LOW); // Set pin 2 to LOW
  }

  for (int i = 0; i < results.num_faces; ++i) {
    const person_sensor_face_t* face = &results.faces[i];
    Serial.print("Face #");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(face->box_confidence);
    Serial.print(" confidence, (");
    Serial.print(face->box_left);
    Serial.print(", ");
    Serial.print(face->box_top);
    Serial.print("), (");
    Serial.print(face->box_right);
    Serial.print(", ");
    Serial.print(face->box_bottom);
    Serial.print("), ");
    if (face->is_facing) {
      Serial.println("facing");
    } else {
      Serial.println("not facing");
    }
  }
  delay(SAMPLE_DELAY_MS);
}
