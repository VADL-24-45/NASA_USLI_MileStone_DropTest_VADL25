#include <Arduino.h>
#include "DataLogger.h"
#include "IMU.h"

// pin declaration
#define SERVO_PIN 4
#define LATCH_PIN 11
// IMU object and local data copy
IMU imu; 
IMU::Data localCopy;
IntervalTimer logTimer;  // Timer to trigger logging

// DataLogger object for SD card logging
DataLogger logger("datalog.csv");

// threshold values
// float timeoutTime 60                          // sec
// float landingMinTime 3                        // sec
float landingAccMagThreshold = 25;               // m/s^2
float GroundLevel = 168;                         // CHANGE THIS VALUE TO CALIBRATE IMU
float landingAltitudeThreshold = GroundLevel + 1.5;
float initialAltitudeThreshold = GroundLevel + 15;

bool ledState = false;  // LED state flag
bool landedState = false;
bool initialAltitudeAchieved = false; 

/**
 * Function prototypes
 */
void logData();
void blinkLED();
void servoInit();
void detectLanding(float altitude, float accel_mag);
void updateServo();

/**
 * @brief Setup function, initializes serial, IMU, SD card, and timer
 */
void setup() 
{
    Serial.begin(115200); // Initialize serial communication
    delay(1000);          // Wait to ensure the serial is ready
    
    pinMode(SERVO_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);


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
    // elapsed_time = millis() - start_time; 
    
    blinkLED();  // Blink onboard LED
    detectLanding(localCopy.localAltitude, localCopy.localAccMag);
    updateServo();
}

/**
 * @brief Callback function for logging data
 * Copies IMU data and logs it to the SD card
 */
void logData()
{
    Serial.print("Time: ");
    Serial.print(float(millis()));  // Print current time in milliseconds
    Serial.print("---");
    Serial.println(localCopy.localAltitude);

    localCopy = imu.imuData;  // Update the local copy of IMU data
    
    // Log data to SD card using IMU data and current time
    logger.logData(float(millis() / 1000.0), 
                   localCopy.localMagX, localCopy.localMagY, localCopy.localMagZ, 
                   localCopy.localAccelX, localCopy.localAccelY, localCopy.localAccelZ, 
                   localCopy.localGyroX, localCopy.localGyroY, localCopy.localGyroZ, 
                   localCopy.localTemperature, localCopy.localPressure, localCopy.localAltitude, 
                   localCopy.localAccMag, localCopy.localQw, localCopy.localQx, localCopy.localQy, localCopy.localQz, 
                   initialAltitudeThreshold, landedState);
}

/**
 * @brief Blinks the onboard LED at a 500ms interval
 */
void blinkLED()
{
    static unsigned long lastBlinkTime = 0;  // Store the last blink time
    unsigned long currentMillis = millis();  // Get current time in milliseconds

    // Toggle the LED state every 500 milliseconds
    if (!landedState && currentMillis - lastBlinkTime >= 500) {
        lastBlinkTime = currentMillis;  // Update the last blink time
        ledState = !ledState;           // Toggle LED state
        digitalWrite(13, ledState);     // Set the LED to the new state
    }
    else if (landedState) {
        digitalWrite(13, HIGH);
    }
}

/**
 * @brief Landing detection function
 * Uses altitude and acceleration to determine the state of the payload
 */
void detectLanding(float altitude, float accel_mag) {
    // timeoutTime used in future (need to add elapsed_time to function header) 
    // if (elapsed_time > timeoutTime) {
    //     landedState = true; 
    // }

    if (altitude > initialAltitudeThreshold) {
        initialAltitudeAchieved = true; 
    }
    
    // update this IF statement in the future to use pdsMinTime (elapsed_time > pdsMinTime)
    if (initialAltitudeAchieved && altitude < landingAltitudeThreshold && accel_mag > landingAccMagThreshold) {
        landedState = true;
    }
}

void updateServo()
{
    if (!landedState)
    {
        digitalWrite(LATCH_PIN, LOW);
        // Generate PWM for 0 degrees
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(1300);  // 1 ms pulse width for 0 degrees
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(18700);  // Complete the 20 ms period (20 ms - 1 ms = 19 ms)
        delay(1000);  // Wait for a second
    }
    else
    {
        digitalWrite(LATCH_PIN, HIGH);
        // Generate PWM for 90 degrees
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(2000);  // 1.5 ms pulse width for 90 degrees
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(18000);  // Complete the 20 ms period (20 ms - 1.5 ms = 18.5 ms)
        delay(1000);  // Wait for a second
    }
}
