#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>

#define IR_PIN1 A1          // IR sensor connected to analog pin A1
#define IR_PIN2 A2
#define TF_LUNA_RX 3        // Connect to TF-Luna TX
#define TF_LUNA_TX 2        // Connect to TF-Luna RX
#define SERVO_PIN 9         // Connect to the servo motor
#define TOTAL_DEGREES 60    // Max range for servo movement
#define MOVE_DELAY 200      // Time in milliseconds to wait after myServo.write
#define LIDAR_READ_DELAY 12 // Time delay between lidar readings (non-blocking)
#define IR_THRESHOLD 500
#define MAX_READINGS 100    // Maximum number of readings to store

SoftwareSerial tfLunaSerial(TF_LUNA_RX, TF_LUNA_TX); // RX, TX
Servo myServo; // Create a servo object

unsigned long lastLidarReadTime = 0;  // To track lidar reading intervals
bool moving = true;
bool goingRight = true;
unsigned long sweepFinishTime = 0;
unsigned long sweepStartTime = 0;
unsigned long sweepTime = 0;  // Total sweep time

// Struct to hold a pair of time and distance
struct Reading {
  unsigned long time; // Time when the reading was acquired
  float distance;     // Lidar distance reading
};

// Array to store readings (time and distance)
Reading lidarReadings[MAX_READINGS];
int lidarCount = 0;  // Number of readings stored in the array

void setup() {
    Serial.begin(115200);        // Initialize serial monitor for debugging
    tfLunaSerial.begin(115200);  // Initialize serial communication with TF-Luna
    myServo.attach(SERVO_PIN);   // Attach the servo to the defined pin
    Serial.println("TF-Luna UART Test");

    myServo.write(0);  // Start at 0 degrees
    delay(500);  // Allow time to stabilize
    sweepStartTime = millis();  // Record start time of the sweep
}

float getLidarReading() {
    // Only attempt to read when we have at least 9 bytes
    if (tfLunaSerial.available() >= 9) {
        uint8_t data[9] = {0}; // Buffer to store the incoming data
        tfLunaSerial.readBytes(data, 9); // Read the data from TF-Luna

        // Check the header for valid data (first byte should be 0x59)
        if (data[0] == 0x59) {
            return ((data[2] << 8) | data[1]) / 279.13;  // Convert distance value
        }
    }
    return -1;  // Return -1 if no valid data
}

// Function to store Lidar readings (time and distance)
void storeLidarReading(unsigned long time, float reading) {
    if (lidarCount < MAX_READINGS) {
        lidarReadings[lidarCount].time = time;      // Store the timestamp
        lidarReadings[lidarCount].distance = reading;  // Store the distance
        lidarCount++;  // Increment the count of readings
    } else {
        Serial.println("Lidar readings array is full");
    }
}

// Function to clear the array when sweep is complete
void clearLidarReadings() {
    lidarCount = 0;  // Reset the count to zero (clearing the array)
}

void processSweepData() {
    // Calculate total sweep time
    sweepTime = sweepFinishTime - sweepStartTime;
    Serial.print("Sweep Time: ");
    Serial.println(sweepTime);

    if (lidarCount > 0) {
        Serial.println("Lidar readings with calculated angles:");
        for (int i = 0; i < lidarCount; i++) {

            // Calculate the angle for each reading
            float timeSinceStart = lidarReadings[i].time - sweepStartTime;
            float angle = (timeSinceStart / (float)sweepTime) * TOTAL_DEGREES;  // Calculate angle based on time

            // Print time, angle, and distance
            Serial.print("Time: ");
            Serial.print(lidarReadings[i].time);
            Serial.print(", Angle: ");
            Serial.print(angle);
            Serial.print(", Distance: ");
            Serial.println(lidarReadings[i].distance);
        }
        clearLidarReadings();  // Clear the array after processing
    } else {
        Serial.println("No readings gathered during sweep");
    }
}

void loop() {
    unsigned long currentTime = millis();

    // Non-blocking Lidar reading check
    if (currentTime - lastLidarReadTime >= LIDAR_READ_DELAY) {
        if (tfLunaSerial.available() >= 9) {
            lastLidarReadTime = millis();  // Update the time of the last reading
            float lidarDistance = getLidarReading();
            if (lidarDistance != -1) {
                storeLidarReading(millis(), lidarDistance);  // Store time and valid reading
            }
        }
    }

    int IRStart = analogRead(IR_PIN1);
    int IREnd = analogRead(IR_PIN2);

    // Move servo based on IR sensor input
    if (IRStart < IR_THRESHOLD) {
      if (moving == true) {
        sweepFinishTime = millis();  // Record the time the sweep finishes
        processSweepData();  // Process the readings and calculate angles
        myServo.write(TOTAL_DEGREES);  // Move servo to 60 degrees
        sweepStartTime = millis();  // Reset start time for the next sweep
        moving = false;
        goingRight = false;
      }
    } else if (IREnd < IR_THRESHOLD) {
      if (goingRight == false && moving == true) {
        sweepFinishTime = millis();  // Record the time the sweep finishes
        processSweepData();  // Process the readings and calculate angles
        myServo.write(0);  // Move servo back to 0 degrees
        sweepStartTime = millis();  // Reset start time for the next sweep
        moving = false;
        goingRight = true;
      }
    } else {
      moving = true;
    }
}
