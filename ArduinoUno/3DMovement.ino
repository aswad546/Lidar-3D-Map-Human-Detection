#include <Servo.h>

Servo xAxisServo; // X-axis servo
Servo yAxisServo; // Y-axis servo

int xAxisPin = 9; // X-axis servo connected to pin 9
int yAxisPin = 5; // Y-axis servo connected to pin 5

int xPos = 0;     // Position of X-axis servo
int yPos = 0;     // Position of Y-axis servo

int xIncrement = 2; // Increment for X-axis movement
int yIncrement = 2; // Increment for Y-axis movement

bool yDirectionForward = true; // To track Y-axis direction

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud
  xAxisServo.attach(xAxisPin); // Attach the X-axis servo
  yAxisServo.attach(yAxisPin); // Attach the Y-axis servo

  xAxisServo.write(xPos);      // Start X-axis at 0 degrees
  yAxisServo.write(yPos);      // Start Y-axis at 0 degrees

  // Print initial position
  logServoPosition();
}

void loop() {
  // Move the X-axis servo from 0 to 60 degrees in increments of 2
  for (xPos = 0; xPos <= 60; xPos += xIncrement) {
    xAxisServo.write(xPos);
    delay(50); // Delay to slow down the movement for observation
    logServoPosition(); // Log the current position
  }

  // Increase or decrease Y-axis position based on direction
  if (yDirectionForward) {
    yPos += yIncrement;
    if (yPos >= 60) {
      yPos = 60;           // Limit Y-axis to 60 degrees
      yDirectionForward = false; // Reverse direction after reaching 60 degrees
    }
  } else {
    yPos -= yIncrement;
    if (yPos <= 0) {
      yPos = 0;            // Limit Y-axis to 0 degrees
      yDirectionForward = true;  // Reverse direction after reaching 0 degrees
    }
  }
  yAxisServo.write(yPos);
  delay(500); // Delay before sweeping back
  logServoPosition(); // Log the current position

  // Move the X-axis servo from 60 to 0 degrees in decrements of 2
  for (xPos = 60; xPos >= 0; xPos -= xIncrement) {
    xAxisServo.write(xPos);
    delay(50); // Delay to slow down the movement for observation
    logServoPosition(); // Log the current position
  }

  // Repeat Y-axis movement
  if (yDirectionForward) {
    yPos += yIncrement;
    if (yPos >= 60) {
      yPos = 60;           // Limit Y-axis to 60 degrees
      yDirectionForward = false; // Reverse direction after reaching 60 degrees
    }
  } else {
    yPos -= yIncrement;
    if (yPos <= 0) {
      yPos = 0;            // Limit Y-axis to 0 degrees
      yDirectionForward = true;  // Reverse direction after reaching 0 degrees
    }
  }
  yAxisServo.write(yPos);
  delay(50); // Delay before sweeping forward again
  logServoPosition(); // Log the current position
}

// Function to log the current servo positions with timestamps
void logServoPosition() {
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  Serial.print("Timestamp: ");
  Serial.print(currentTime);
  Serial.print(" ms, X-axis: ");
  Serial.print(xPos);
  Serial.print(" degrees, Y-axis: ");
  Serial.print(yPos);
  Serial.println(" degrees");
}
