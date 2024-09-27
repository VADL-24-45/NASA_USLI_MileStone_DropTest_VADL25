#include <Arduino.h>
#include "DataLogger.h"
#include "IMU.h"

IMU imu; 
IMU::Data localCopy;

// Create a DataLogger object
DataLogger logger("datalog.csv");

void setup() {
    Serial.begin(115200);
    delay(1000);

    imu.begin();

    // Initialize the SD card
    if (!logger.begin()) {
        Serial.println("SD Card initialization failed!");
        return;
    }

    // Write the header to the file
    logger.writeHeader();

    Serial.println("**Setup Ready**");
}

void loop() {
// Update copy
localCopy = imu.imuData;  

// Log the data to the SD card using elements from the Data struct
logger.logData(float(millis()/float(1000)), 
               localCopy.localMagX, localCopy.localMagY, localCopy.localMagZ, 
               localCopy.localAccelX, localCopy.localAccelY, localCopy.localAccelZ, 
               localCopy.localGyroX, localCopy.localGyroY, localCopy.localGyroZ, 
               localCopy.localTemperature, localCopy.localPressure, localCopy.localAltitude, 
               localCopy.localAccMag, localCopy.localQw, localCopy.localQx, localCopy.localQy, localCopy.localQz);

Serial.println(millis());
}
