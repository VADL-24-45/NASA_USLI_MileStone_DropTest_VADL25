#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <SD.h>
#include <SPI.h>

/**
 * @class DataLogger
 * @brief Handles logging of IMU and environmental sensor data to an SD card.
 */
class DataLogger {
public:
    /**
     * @brief Constructs a DataLogger object with the given filename and optional chip select pin.
     * @param filename The name of the file to log data into.
     * @param chipSelectPin The chip select pin for the SD card, default is 254.
     */
    DataLogger(const char* filename, int chipSelectPin = 254);

    /**
     * @brief Initializes the SD card and prepares it for data logging. Clear files.
     * @return True if the SD card was successfully initialized, otherwise false.
     * @note This function must be called before attempting to log any data.
     */
    bool begin();

    /**
     * @brief Writes a header to the log file, typically used to describe the data format.
     * @return None
     * @note This should be called once, usually after initializing the logger, to label the data columns.
     */
    void writeHeader();

    /**
     * @brief Logs IMU and sensor data to the file.
     * @param time The timestamp of the data.
     * @param magX Magnetometer X-axis data.
     * @param magY Magnetometer Y-axis data.
     * @param magZ Magnetometer Z-axis data.
     * @param accelX Accelerometer X-axis data.
     * @param accelY Accelerometer Y-axis data.
     * @param accelZ Accelerometer Z-axis data.
     * @param gyroX Gyroscope X-axis data.
     * @param gyroY Gyroscope Y-axis data.
     * @param gyroZ Gyroscope Z-axis data.
     * @param temperature Temperature sensor data.
     * @param pressure Pressure sensor data.
     * @param altitude Altitude calculated from pressure.
     * @param accMag Magnitude of the acceleration vector.
     * @param qw Quaternion W component.
     * @param qx Quaternion X component.
     * @param qy Quaternion Y component.
     * @param qz Quaternion Z component.
     * @return None
     * @note This function logs all sensor data in a single line, formatted to the file.
     */
    void logData(float time, float magX, float magY, float magZ, 
                 float accelX, float accelY, float accelZ, 
                 float gyroX, float gyroY, float gyroZ, 
                 float temperature, float pressure, float altitude, 
                 float accMag, float qw, float qx, float qy, float qz);

private:
    const char* filename;  ///< The name of the file used for logging data.
    int chipSelectPin;     ///< Chip select pin used for SD card communication.
    File myFile;           ///< File object used to manage SD card file operations.
};

#endif
