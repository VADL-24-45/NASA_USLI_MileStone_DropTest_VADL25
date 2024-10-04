#include "IMU.h"

IMU::IMU() : requestRaw(true), requestQuaternion(false) {
    stringComplete = false;
    dataReady = false;
    input = "";
}


void IMU::begin() {
    Serial3.begin(115200);  // Initial baud rate
    delay(1000);

    initializeBaud();
    Serial.println("IMU Initialized");

    requestTimer.begin([this]() { this->requestData(); }, 2500);  // 100 Hz
    updateTimer.begin([this]() { this->updateData(); }, 2500);    // 100 Hz
    processTimer.begin([this]() { this->processIMUData(); }, 2000);
}


void IMU::flushMessage() {
    Serial3.print("$VNWRG,54,0*XX\r\n");
    Serial3.print("$VNWRG,09,0*XX\r\n");
    Serial3.print("$VNASY,0*XX\r\n");
    delay(100);
    while (Serial3.available() > 0) {
        Serial3.read();
    }
}


void IMU::initializeBaud() {
    Serial3.begin(115200);
    delay(1000);
    flushMessage();
    Serial3.print("$VNWRG,05,460800*XX\r\n");
    delay(1000);
    Serial3.end();
    Serial3.begin(460800);
}


void IMU::requestData() {
    noInterrupts();
    if (requestRaw) {
        Serial3.print("$VNRRG,54,1*XX\r\n");
        requestRaw = false;
        requestQuaternion = true;
    } else if (requestQuaternion) {
        Serial3.print("$VNRRG,09,1*XX\r\n");
        requestRaw = true;
        requestQuaternion = false;
    }
    interrupts();
}


void IMU::processIMUData() {
    noInterrupts();
    while (Serial3.available() > 0) {
        char c = Serial3.read();
        input += c;
        if (c == '\n') {
            stringComplete = true;
        }
    }

    if (stringComplete) {
        if (input.startsWith("$VNRRG,54")) {
            String data = input.substring(10);
            data = data.substring(0, data.indexOf('*'));
            magX = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            magY = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            magZ = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            accelX = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            accelY = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            accelZ = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            gyroX = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            gyroY = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            gyroZ = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            temperature = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            pressure = data.toFloat();
            dataReady = true;
        }
        
        if (input.startsWith("$VNRRG,09")) {
            String data = input.substring(10);
            data = data.substring(0, data.indexOf('*'));
            qw = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            qx = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            qy = data.substring(0, data.indexOf(',')).toFloat();
            data = data.substring(data.indexOf(',') + 1);
            qz = data.toFloat();
            dataReady = true;
        }

        input = "";
        stringComplete = false;
    }
    interrupts();
}


void IMU::updateData() {
    if (dataReady) {
        noInterrupts();
        imuData.localMagX = magX;
        imuData.localMagY = magY;
        imuData.localMagZ = magZ;
        imuData.localAccelX = accelX;
        imuData.localAccelY = accelY;
        imuData.localAccelZ = accelZ;
        imuData.localGyroX = gyroX;
        imuData.localGyroY = gyroY;
        imuData.localGyroZ = gyroZ;
        imuData.localTemperature = temperature;
        imuData.localPressure = pressure;
        imuData.localQw = qw;
        imuData.localQx = qx;
        imuData.localQy = qy;
        imuData.localQz = qz;
        
        // Calculate altitude and acceleration magnitude
        imuData.localAltitude = calculateAltitude(imuData.localPressure); 
        imuData.localAccMag = getAccelerationMagnitude(); 

        dataReady = false;
        interrupts();
    }
}

// Function to calculate and return the magnitude of acceleration
float IMU::getAccelerationMagnitude() const {
    // Use the local accelerometer values
    float ax = imuData.localAccelX;
    float ay = imuData.localAccelY;
    float az = imuData.localAccelZ;

    // Calculate the magnitude of the acceleration vector
    return sqrt(ax * ax + ay * ay + az * az);
}


float IMU::calculateAltitude(float pressure) const {
    static float buffer[20] = {0};
    static int bufferIdx = 0;

    const float seaLevelPressure = 101.325;  // Standard atmospheric pressure at sea level in hPa
    float currentAltitude = 44330.0 * (1.0 - pow(pressure / seaLevelPressure, 0.1903));

    buffer[bufferIdx] = currentAltitude;


    float averageAltitude = 0;
    for (int i = 0; i < 20; i ++)
    {
        averageAltitude += buffer[i];
    }

    averageAltitude = averageAltitude / 20.0;

    bufferIdx = (bufferIdx + 1) % 20;

    return averageAltitude;
}
