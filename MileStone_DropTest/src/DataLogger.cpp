#include "DataLogger.h"

// Constructor
DataLogger::DataLogger(const char* filename, int chipSelectPin)
    : filename(filename), chipSelectPin(chipSelectPin) {}

// Function to initialize the SD card
bool DataLogger::begin() {
    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelectPin)) {
        Serial.println("initialization failed!");
        return false;
    }
    Serial.println("initialization done.");

    // Clear File each time running
    if (SD.exists(filename))
    {
        SD.remove(filename);
    }
    return true;
}   

// Function to write the header
void DataLogger::writeHeader() {
    myFile = SD.open(filename, FILE_WRITE);
    if (myFile) {
        myFile.println("time,magX,magY,magZ,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,temperature,pressure,altitude,accMag,qw,qx,qy,qz,initialAltAchieved,landingDetected");
        myFile.close();
    }
}

// Function to log data
void DataLogger::logData(float time, float magX, float magY, float magZ, 
                         float accelX, float accelY, float accelZ, 
                         float gyroX, float gyroY, float gyroZ, 
                         float temperature, float pressure, float altitude, 
                         float accMag, float qw, float qx, float qy, float qz, bool initialAltAchieved, bool landingDetected) {

    myFile = SD.open(filename, FILE_WRITE);
    if (myFile) {
        myFile.print(time); myFile.print(", ");
        myFile.print(magX); myFile.print(", ");
        myFile.print(magY); myFile.print(", ");
        myFile.print(magZ); myFile.print(", ");
        myFile.print(accelX); myFile.print(", ");
        myFile.print(accelY); myFile.print(", ");
        myFile.print(accelZ); myFile.print(", ");
        myFile.print(gyroX); myFile.print(", ");
        myFile.print(gyroY); myFile.print(", ");
        myFile.print(gyroZ); myFile.print(", ");
        myFile.print(temperature); myFile.print(", ");
        myFile.print(pressure); myFile.print(", ");
        myFile.print(altitude); myFile.print(", ");
        myFile.print(accMag); myFile.print(", ");
        myFile.print(qw); myFile.print(", ");
        myFile.print(qx); myFile.print(", ");
        myFile.print(qy); myFile.print(", ");
        myFile.print(qz); myFile.print(", ");
        myFile.print(initialAltAchieved); myFile.print(", ");
        myFile.println(landingDetected); 
        myFile.close();
    }
}
