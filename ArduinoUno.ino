#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>

#define IR_PIN1 A1          // IR sensor connected to analog pin A1
#define IR_PIN2 A2
#define TF_LUNA_RX 3   // Connect to TF-Luna TX
#define TF_LUNA_TX 2   // Connect to TF-Luna RX
#define SERVO_PIN 9    // Connect to the servo motor
#define TOTAL_DEGREES 60  // Max range for servo movement
#define MOVE_DELAY 200  // Time in milliseconds to wait after myServo.write
#define LIDAR_READ_DELAY 8  // Time delay between lidar readings
#define IR_THRESHOLD 500

SoftwareSerial tfLunaSerial(TF_LUNA_RX, TF_LUNA_TX); // RX, TX
Servo myServo; // Create a servo object

unsigned long lastLidarReadTime = 0;  // To track lidar reading intervals
unsigned long cutTime = 0; // To track when the servo motor was last at 0°
bool moving = true;
bool goingRight = true;

void setup() {
    Serial.begin(115200);        // Initialize serial monitor for debugging
    tfLunaSerial.begin(115200);  // Initialize serial communication with TF-Luna
    myServo.attach(SERVO_PIN);   // Attach the servo to the defined pin
    Serial.println("TF-Luna UART Test");

    myServo.write(0);  // Start at 0 degrees
    delay(500);  // Allow time to stabilize
}

float getLidarReading() {
    if (tfLunaSerial.available() >= 9) { // Wait until 9 bytes are available
        uint8_t data[9] = {0}; // Buffer to store the incoming data
        tfLunaSerial.readBytes(data, 9); // Read the data from TF-Luna
        // Check the header for valid data (first byte should be 0x59)
        if (data[0] == 0x59) {
            // Extract distance value from the data and divide by 279.13
            return ((data[2] << 8) | data[1]) / 279.13; 
        }
    }
    return -1;  // Return -1 if no valid data
}

// Function to calculate x, y, z coordinates based on distance and angle
void calculateCoordinates(float distance, int degree) {
    float theta = degree * (PI / 180.0);  // Convert degrees to radians
    float x = distance * cos(theta);      // Calculate x-coordinate
    float y = distance * sin(theta);      // Calculate y-coordinate
    float z = 0;                          // Assuming no vertical movement for now

    // Print the coordinates
    // Serial.print("X: ");
    // Serial.print(x);
    // Serial.print(", Y: ");
    // Serial.print(y);
    // Serial.print(", Z: ");
    // Serial.println(z);
}

void loop() {
    unsigned long currentTime = millis();
    int IRStart = analogRead(IR_PIN1);
    int IREnd = analogRead(IR_PIN2);
    if (IRStart < IR_THRESHOLD) {
      if (goingRight == true && moving == true) {
        myServo.write(TOTAL_DEGREES);
        moving = false;
        goingRight = false;
      }
    } else if (IREnd < IR_THRESHOLD) {
      if (goingRight == false && moving == true) {
        myServo.write(0);
        moving = false;
        goingRight = true;
      }
    } else {
      moving = true;
    }


    // Read from the lidar at regular intervals
    if (currentTime - lastLidarReadTime >= LIDAR_READ_DELAY) {
        lastLidarReadTime = currentTime;  // Update the last read time

        float distance = getLidarReading();  // Take lidar reading
        if (distance != -1) {
            int currentServoPosition = moveToMax ? TOTAL_DEGREES : 0;
            
            // Serial.print("Current Servo Position: ");
            // Serial.print(currentServoPosition);
            // Serial.print(", Distance: ");
            // Serial.print(distance);
            // Serial.println(" cm");

            // Calculate and display the x, y, z coordinates
            calculateCoordinates(distance, currentServoPosition);
        }
    }
}
