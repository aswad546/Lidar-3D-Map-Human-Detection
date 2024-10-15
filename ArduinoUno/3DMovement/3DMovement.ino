#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <math.h>

#define TF_LUNA_RX 3        // Connect to TF-Luna TX
#define TF_LUNA_TX 2        // Connect to TF-Luna RX


Servo xAxisServo; // X-axis servo
Servo yAxisServo; // Y-axis servo

SoftwareSerial tfLunaSerial(TF_LUNA_RX, TF_LUNA_TX); // RX, TX

int xAxisPin = 10; // X-axis servo connected to pin 9
int yAxisPin = 11; // Y-axis servo connected to pin 5

int horizontalDegree = 0;     // Position of X-axis servo
int verticalDegree = 0;     // Position of Y-axis servo

int xIncrement = 2; // Increment for X-axis movement
int yIncrement = 2; // Increment for Y-axis movement

unsigned long lastServoUpdate = 0;
const unsigned long servoInterval = 160;  // Adjust the interval for servo updates

bool yDirectionForward = true; // To track Y-axis direction
bool horizontalSweepComplete = false;
bool verticalSweepComplete = false;
bool horizontalSweepActive = true;
bool verticalSweepActive = false;

int currentXPos = 0;
int currentYPos = 0;

void setup() {
  Serial.begin(115200); // Start serial communication at 115200 baud 

  // Step 1: Start communication with TF-Luna at its default baud rate (115200)
  tfLunaSerial.begin(115200);  // Start SoftwareSerial at TF-Luna's default baud rate (115200)

  // // Step 2: Send command to change TF-Luna baud rate to 9600
  // uint8_t changeBaudRateCommand[8] = {0x5A, 0x08, 0x06, 0x80, 0x25, 0x00, 0x00, 0x00};
  // tfLunaSerial.write(changeBaudRateCommand, 8);  // Send the baud rate change command
  
  // delay(100);  // Give the sensor time to process the command

  // // Step 3: Save the new baud rate setting to TF-Luna's memory
  // uint8_t saveSettingsCommand[4] = {0x5A, 0x04, 0x11, 0x00};
  // tfLunaSerial.write(saveSettingsCommand, 4);  // Send the save settings command
  
  // delay(100);  // Give the sensor time to save the setting

  // // Step 4: End communication at 115200 and restart at 9600 baud
  // tfLunaSerial.end();  // End the current SoftwareSerial session
  // delay(1000);  // Wait 1 second for the sensor to adjust


  // tfLunaSerial.begin(9600);
  xAxisServo.attach(xAxisPin); // Attach the X-axis servo
  yAxisServo.attach(yAxisPin); // Attach the Y-axis servo
  Serial.println("TF-Luna 3D Demo");
  xAxisServo.write(currentXPos);      // Start X-axis at 0 degrees
  yAxisServo.write(currentYPos);      // Start Y-axis at 0 degrees
}

float getLidarReading() {
    // Only attempt to read when we have at least 9 bytes
    if (tfLunaSerial.available() >= 9) {
        uint8_t data[9] = {0}; // Buffer to store the incoming data
        tfLunaSerial.readBytes(data, 9); // Read the data from TF-Luna

        // Check the header for valid data (first byte should be 0x59)
        if (data[0] == 0x59) {
            tfLunaSerial.flush();
            return ((data[2] << 8) | data[1]) / 279.13;  // Convert distance value
        } 
    }
    return -1;  // Return -1 if no valid data
}

uint8_t lidarData[9];  // Buffer to store the 9 bytes
int lidarBytesRead = 0;  // Track how many bytes have been read so far

float getLidarReadingNonBlocking() {
    // Read bytes from the sensor if available
    while (tfLunaSerial.available() > 0) {
        lidarData[lidarBytesRead] = tfLunaSerial.read();  // Read one byte at a time
        lidarBytesRead++;

        // Once we've read 9 bytes, process the data
        if (lidarBytesRead == 9) {
            lidarBytesRead = 0;  // Reset for the next reading

            // Check if the first byte is the correct header
            if (lidarData[0] == 0x59) {
                return ((lidarData[2] << 8) | lidarData[1]) / 279.13;  // Return distance
            } else {
                return -1;  // Invalid data
            }
        }
    }

    return -1;  // Still waiting for more bytes
}



void loop() {
  float lidarReading = -1;
  unsigned long currentMillis = millis();
  if (currentMillis - lastServoUpdate >= servoInterval) {
    lastServoUpdate = currentMillis;
    if (horizontalSweepActive) {
      // Keep track of when the horizontal sweep completes to flip sweep direction
      horizontalSweepComplete = currentXPos == 60 ? true : currentXPos == 0 ? false : horizontalSweepComplete;
      // If the horizontal motor is set to 0 or 60 degrees move the vertical motor in the next iteration
      verticalSweepActive = currentXPos == 60 || currentXPos == 0 ? true : false;
      // The motor moves horizontally or vertically in one step
      horizontalSweepActive = !verticalSweepActive;
      // Read from lidar sensor
      lidarReading = getLidarReading();
      // Command the motor to go to the next horizontal position
      xAxisServo.write(currentXPos); 
      // Set the next movement of the motor based on if a cycle has been complete or not.
      currentXPos += horizontalSweepComplete ? 0 : 0;
    }
    else if (verticalSweepActive) {
      // Toggle verticalSweep to stop until triggered again after one full horizontal cycle
      verticalSweepActive = false;
      // Reactivate horizontal motion in the next step
      horizontalSweepActive = !verticalSweepActive;
      // Keep track of when the vertical motor reaches its peak
      verticalSweepComplete = currentYPos == 60 ? true : currentYPos == 0 ? false : verticalSweepComplete;
      // Read from lidar sensor
      lidarReading = getLidarReading();
      
      // Command the motor to go to the next vertical position
      yAxisServo.write(currentYPos);
      // Set the next movement of the motor based on if a cycle has been complete or not.
      currentYPos += verticalSweepComplete ? 0 : 0;
    }
    logServoPosition(lidarReading);
  }
}

// Function to log the current servo positions with timestamps
void logServoPosition(float lidarReading) {
  unsigned long currentTime = millis(); // Get the current time in milliseconds
  Serial.print("Timestamp: ");
  Serial.print(currentTime);
  Serial.print(" ms, X-axis: ");
  Serial.print(currentXPos);
  Serial.print(" degrees, Y-axis: ");
  Serial.print(currentYPos);
  Serial.println(" degrees");
  Serial.print("Distance: ");
  Serial.println(lidarReading);
}
