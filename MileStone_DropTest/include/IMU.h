#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <IntervalTimer.h>

/**
 * @class IMU
 * @brief A class for managing communication and data processing from the IMU sensor.
 */
class IMU {
public:
    /**
     * @brief IMU class constructor.
     * @return None
     */
    IMU();

    /**
     * @brief Initializes the IMU communication and starts the required timers.
     * @return None
     * @note This function sets up the serial connection to the IMU and prepares it to request and process data.
     */
    void begin();

    /**
     * @brief Requests data from the IMU based on the current mode (raw or quaternion).
     * @return None
     * @note Alternates between requesting raw data and quaternion data.
     */
    void requestData();

    /**
     * @brief Processes the IMU data received through the serial interface.
     * @return None
     * @note This function parses the received data and updates the internal buffers.
     */
    void processIMUData();

    /**
     * @brief Updates local IMU data copies and computes altitude and acceleration magnitude.
     * @return None
     * @note This function stores IMU data in local variables and calculates values like altitude and acceleration magnitude.
     */
    void updateData();

    /**
     * @brief Retrieves the quaternion values.
     * @param [out] qw Reference to store quaternion W value.
     * @param [out] qx Reference to store quaternion X value.
     * @param [out] qy Reference to store quaternion Y value.
     * @param [out] qz Reference to store quaternion Z value.
     * @return None
     * @note This function returns the quaternion values obtained from the IMU.
     */
    void getQuaternions(float& qw, float& qx, float& qy, float& qz) const;

    /**
     * @brief Calculates the magnitude of the acceleration vector.
     * @return The total magnitude of acceleration in meters per second squared (m/s²).
     * @note This function computes the magnitude using the accelerometer data (accelX, accelY, accelZ).
     */
    float getAccelerationMagnitude() const;

    /**
     * @brief Calculates altitude based on atmospheric pressure.
     * @param [in] pressure The measured atmospheric pressure in hPa.
     * @return The altitude in meters above sea level.
     * @note This function uses the barometric formula to calculate altitude from the measured pressure.
     */
    float calculateAltitude(float pressure) const;

    /**
     * @struct Data
     * @brief A structure to hold all IMU sensor data and local copies to prevent race conditions.
     */
    struct Data {
        // Local copies of the data to avoid race conditions
        float localMagX;      ///< Local magnetometer X value
        float localMagY;      ///< Local magnetometer Y value
        float localMagZ;      ///< Local magnetometer Z value
        float localAccelX;    ///< Local accelerometer X value
        float localAccelY;    ///< Local accelerometer Y value
        float localAccelZ;    ///< Local accelerometer Z value
        float localGyroX;     ///< Local gyroscope X value
        float localGyroY;     ///< Local gyroscope Y value
        float localGyroZ;     ///< Local gyroscope Z value
        float localTemperature;   ///< Local temperature value
        unsigned long localPressure;      ///< Local atmospheric pressure
        float localAltitude;      ///< Local altitude value calculated from pressure
        float localAccMag;        ///< Local total magnitude of acceleration
        float localQw;            ///< Local quaternion W value
        float localQx;            ///< Local quaternion X value
        float localQy;            ///< Local quaternion Y value
        float localQz;            ///< Local quaternion Z value
    };

    Data imuData;  ///< Public instance of the Data struct to hold all IMU-related data

private:
    /**
     * @brief Initializes the IMU baud rate and flushes any previous messages.
     * @return None
     * @note This function sets up the communication speed with the IMU.
     */
    void initializeBaud();

    /**
     * @brief Flushes old messages from the IMU to prevent data corruption.
     * @return None
     */
    void flushMessage();

    bool requestRaw;         ///< Flag indicating if raw data is being requested
    bool requestQuaternion;  ///< Flag indicating if quaternion data is being requested

    String input; ///< Buffer to hold input data from the IMU

    // Timers for controlling IMU data requests and processing
    IntervalTimer requestTimer;
    IntervalTimer updateTimer;
    IntervalTimer processTimer;

    volatile float magX;    ///< Magnetometer X value
    volatile float magY;    ///< Magnetometer Y value
    volatile float magZ;    ///< Magnetometer Z value
    volatile float accelX;  ///< Accelerometer X value
    volatile float accelY;  ///< Accelerometer Y value
    volatile float accelZ;  ///< Accelerometer Z value
    volatile float gyroX;   ///< Gyroscope X value
    volatile float gyroY;   ///< Gyroscope Y value
    volatile float gyroZ;   ///< Gyroscope Z value
    volatile float qw;      ///< Quaternion W value
    volatile float qx;      ///< Quaternion X value
    volatile float qy;      ///< Quaternion Y value
    volatile float qz;      ///< Quaternion Z value
    volatile float temperature;   ///< Temperature value from the IMU
    volatile float pressure;      ///< Atmospheric pressure in hPa
    volatile float altitude;      ///< Altitude calculated from pressure
    volatile bool stringComplete; ///< Flag indicating if a complete string was received from the IMU
    volatile bool dataReady;      ///< Flag indicating if new data is ready to be processed
};

#endif
