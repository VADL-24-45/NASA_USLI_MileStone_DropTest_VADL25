#include <Arduino.h>
#include "DataLogger.h"
#include "IMU.h"

// IMU object and local data copy
IMU imu; 
IMU::Data localCopy;
IntervalTimer logTimer;  // Timer to trigger logging

// DataLogger object for SD card logging
DataLogger logger("datalog.csv");

bool ledState = false;  // LED state flag

/**
 * Function prototypes
 */
void logData();
void blinkLED();

/**
 * @brief Setup function, initializes serial, IMU, SD card, and timer
 */
void setup() 
{
    Serial.begin(115200); // Initialize serial communication
    delay(1000);          // Wait to ensure the serial is ready

    imu.begin();          // Initialize IMU

    // Initialize SD card for logging
    if (!logger.begin()) {
        Serial.println("SD Card initialization failed!");
        return;  // Exit if SD card initialization fails
    }

    // Write the data header to the log file
    logger.writeHeader();

    // Start the logging timer (100Hz logging)
    logTimer.begin(logData, 10000);

    pinMode(13, OUTPUT);  // Set onboard LED pin as output

    Serial.println("**Setup Ready**");
}

/**
 * @brief Main loop function
 * Continuously prints the current time and handles LED blinking
 */
void loop() 
{
    Serial.print("Time: ");
    Serial.println(float(millis()));  // Print current time in milliseconds

    blinkLED();  // Blink onboard LED
}

/**
 * @brief Callback function for logging data
 * Copies IMU data and logs it to the SD card
 */
void logData()
{
    localCopy = imu.imuData;  // Update the local copy of IMU data
    
    // Log data to SD card using IMU data and current time
    logger.logData(float(millis() / 1000.0), 
                   localCopy.localMagX, localCopy.localMagY, localCopy.localMagZ, 
                   localCopy.localAccelX, localCopy.localAccelY, localCopy.localAccelZ, 
                   localCopy.localGyroX, localCopy.localGyroY, localCopy.localGyroZ, 
                   localCopy.localTemperature, localCopy.localPressure, localCopy.localAltitude, 
                   localCopy.localAccMag, localCopy.localQw, localCopy.localQx, localCopy.localQy, localCopy.localQz);
}

/**
 * @brief Blinks the onboard LED at a 500ms interval
 */
void blinkLED()
{
    static unsigned long lastBlinkTime = 0;  // Store the last blink time
    unsigned long currentMillis = millis();  // Get current time in milliseconds

    // Toggle the LED state every 500 milliseconds
    if (currentMillis - lastBlinkTime >= 500) {
        lastBlinkTime = currentMillis;  // Update the last blink time
        ledState = !ledState;           // Toggle LED state
        digitalWrite(13, ledState);     // Set the LED to the new state
    }
}
