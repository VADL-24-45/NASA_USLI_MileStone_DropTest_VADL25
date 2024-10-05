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
// float timeoutTime 60                       // sec
// float landingMinTime 3                     // sec
float landingAccMagThreshold = 25;               // m/s^2
float GroundLevel = 113;                         // CHANGE THIS VALUE TO CALIBRATE IMU
float landingAltitudeThreshold = GroundLevel + 20;
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
void updateServo(bool landedState);
void detectLanding(float altitude, float accel_mag);
void updateServo(bool landedState);

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

    // Start the logging timer (50Hz logging)
    logTimer.begin(logData, 20000);

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
    updateServo(landedState);
  
    // if(landedState) {
    //   digitalWrite(LANDING_DETECTED, HIGH);
    // }
}

/**
 * @brief Callback function for logging data
 * Copies IMU data and logs it to the SD card
 */
void logData()
{
    Serial.print("Time: ");
    Serial.print(uint64_t(millis())); Serial.print("   "); // Print current time in milliseconds
    // Serial.print("---");
    // Serial.println(localCopy.localAltitude);

    Serial.print(localCopy.localMagX); Serial.print("   ");
    Serial.print(localCopy.localMagY); Serial.print("   ");
    Serial.print(localCopy.localMagZ); Serial.print("   ");
    Serial.print(localCopy.localAccelX); Serial.print("   ");
    Serial.print(localCopy.localAccelY); Serial.print("   ");
    Serial.print(localCopy.localAccelZ); Serial.print("   ");
    Serial.print(localCopy.localGyroX); Serial.print("   ");
    Serial.print(localCopy.localGyroY); Serial.print("   ");
    Serial.print(localCopy.localGyroZ); Serial.print("   ");
    Serial.print(localCopy.localTemperature); Serial.print("   ");
    Serial.print(localCopy.localPressure); Serial.print("   ");
    Serial.print(localCopy.localAltitude); Serial.print("   ");
    Serial.print(localCopy.localAccMag); Serial.print("   ");
    Serial.print(localCopy.localQw); Serial.print("   ");
    Serial.print(localCopy.localQx); Serial.print("   ");
    Serial.print(localCopy.localQy); Serial.print("   ");
    Serial.println(localCopy.localQz);

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

void updateServo(bool landedState)
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


// #include <Arduino.h>
// #include <SD.h>
// #include <SPI.h>
// #include <EEPROM.h>

// auto& VN_Serial = Serial3;

// // receiving & sending ASCII input
// char c;
// String input = "";     // read IMU responses 
// String inputIMU = "";  // save IMU data from register 54
// String inputQT = "";   // save YPR data from register 9
// volatile bool stringComplete = false;
// volatile bool send_IMU = true;   // Start by sending IMU first
// volatile bool send_YPR = false;  // Send YPR after IMU is processed
// volatile bool waitingForResponse = false;  // Ensure we wait for a response

// // Define VectorNav Variables
// #define VNRRG_LEN 10
// int decDigit = 6;

// // sampling frequency in microseconds
// int T1 = 10000; 

// // baud rate for data receiving
// int defaultBaud = 115200;
// int setBaud = 460800;

// //Define conversion factors
// float ft2m = 0.3048; // [m/ft]
// float C2K = 273.15; // Celsius to Kelvin
// float m2km = 0.001;   // [km/m]

// // Define altitude calculation variables
//   float R = 287;        // [kg/JK] universal gas constant
//   float b1 = 6.62*m2km;   // [K/m] variation of temperature within the troposphere
//   float b2 = 5.94*m2km;

//   float P0 = 98.46; //101.325;
//   float T0 = 295.6; //298;
//   float g0 = 9.81;
//   float baselineAltitude;

// // IMU Data structure
// struct imuData {
//   float magX;
//   float magY;
//   float magZ;
//   float accelX;
//   float accelY;
//   float accelZ;
//   float gyroX;
//   float gyroY;
//   float gyroZ;
//   float temperature;
//   float pressure; 
//   float altitude;
//   float qw;
//   float qx;
//   float qy;
//   float qz;
// };
// imuData imu;

// IntervalTimer myTimer;
// float timer;

// // declare functions
// void flushMessage();
// void initializeBaud();
// void sendMessage();
// void parseQT(String data);
// void parseIMU(String data);
// void printData();
// float pressure2Altitude(float P);
// void matlabTest();

// void setup() {
//   Serial.begin(115200);

//   while (!Serial && (millis() < 5000)) {};

//   VN_Serial.begin(defaultBaud); // 115200
//   Serial.println("Serial 1 begin");
//   delay(1000);

//   flushMessage();
//   initializeBaud();

//   // restart Serial3 communication with new baudrate
//   VN_Serial.begin(setBaud); 

//   delay(1000);
//   Serial.println("Begin receiving message");
//   delay(500);

//   Serial.println("accelX, accelY, accelZ, qw, qx, qy, qz, temperature, pressure, altitude");
//   // sends and reads message to vn at a fixed frequency
//   myTimer.begin(sendMessage, T1);
// }

// void loop() {
//   timer = millis();
  
//   // Check if data is available from VN sensor
//   if (VN_Serial.available() > 0) {
//     c = VN_Serial.read();
//     input += c;

//     // If the input message ends with a newline, it's complete
//     if (c == '\n') {
//       stringComplete = true;
//       waitingForResponse = false;  // We received a response, stop waiting

//       // Handle IMU (register 54) message
//       if (input.startsWith("$VNRRG,54")) {
//         inputIMU = input;
//         parseIMU(inputIMU);
//         input = "";
//         // Serial.print(inputIMU);
//         // Serial.println();

//         send_YPR = true;   // Flag to send YPR request next
//         send_IMU = false;  // Don't send IMU request again yet
//       }
//       // Handle YPR (register 9) message
//       else if (input.startsWith("$VNRRG,09")) {
//         inputQT = input;
//         parseQT(inputQT);
//         input = "";
//         // Serial.print(inputQT);
//         // Serial.println();

//         send_IMU = true;  // Flag to send IMU request next
//         send_YPR = false; // Don't send YPR request again yet
//       }

//       input = "";  // Clear input after handling
//       stringComplete = false;
//       // printData();
//       matlabTest();
//     }
//   }
// }

// // Flush old messages from the serial buffer
// void flushMessage(){
//   VN_Serial.print("$VNWRG,54,0*XX\r\n"); // IMU
//   VN_Serial.print("$VNASY,0*XX\r\n");    // Stops async data output
//   delay(100);
//   // Flush IMU
//   for (int i = 0; i < 150; i++) {
//     VN_Serial.read();
//   }
//   Serial.println("IMU initialization DONE");
// }

// void initializeBaud(){
//   VN_Serial.print("$VNWRG,05,460800*XX\r\n"); // set new baudrate
//   delay(1000);

//   while (VN_Serial.available() > 0) {
//     c = VN_Serial.read();
//     input += c;
//   }

//   if (input == "") {
//     Serial.println("No IMU baud response received");
//   } else {
//     Serial.print("IMU Baud rate response: "); Serial.println(input);
//   }
  
//   input = "";

//   VN_Serial.end();
//   delay(1000);
// }

// // Send requests at fixed intervals
// void sendMessage() {
//   // Only send a new message if we're not still waiting for a previous response
//   if (!waitingForResponse) {
//     if (send_IMU) {
//       // Send IMU (register 54) request
//       VN_Serial.print("$VNRRG,54,1*XX\r\n");
//       waitingForResponse = true;  // Now wait for IMU response
//     } else if (send_YPR) {
//       // Send YPR (register 9) request
//       VN_Serial.print("$VNRRG,09,1*XX\r\n");
//       waitingForResponse = true;  // Now wait for YPR response
//     }
//   } 
// }

// float pressure2Altitude(float P) {
//     return T0/b1*(pow(P/P0,-R*b2/g0) - 0.9946);
// }

// void printData(){
//   Serial.print("AccelX: "); Serial.println(imu.accelX, decDigit);
//   Serial.print("AccelY: "); Serial.println(imu.accelY, decDigit);
//   Serial.print("AccelZ: "); Serial.println(imu.accelZ, decDigit);

//   Serial.print("Quaternion W: "); Serial.println(imu.qw, decDigit);
//   Serial.print("Quaternion X: "); Serial.println(imu.qx, decDigit);
//   Serial.print("Quaternion Y: "); Serial.println(imu.qy, decDigit);
//   Serial.print("Quaternion Z: "); Serial.println(imu.qz, decDigit);

//   Serial.print("Temperature: "); Serial.println(imu.temperature, decDigit);
//   Serial.print("Pressure: "); Serial.println(imu.pressure, decDigit);
//   imu.altitude = pressure2Altitude(imu.pressure);
//   Serial.print("Altitude: "); Serial.println(imu.altitude, decDigit);

//   Serial.println();
// }

// void matlabTest(){
//   Serial.print(imu.accelX, decDigit); Serial.print(",");
//   Serial.print(imu.accelY, decDigit); Serial.print(",");
//   Serial.print(imu.accelZ, decDigit); Serial.print(",");
//   Serial.print(imu.qw, decDigit); Serial.print(",");
//   Serial.print(imu.qx, decDigit); Serial.print(",");
//   Serial.print(imu.qy, decDigit); Serial.print(",");
//   Serial.print(imu.qz, decDigit); Serial.print(",");
//   Serial.print(imu.temperature, decDigit); Serial.print(",");
//   Serial.print(imu.pressure, decDigit); Serial.print(",");
//     imu.altitude = pressure2Altitude(imu.pressure);
//   Serial.print(imu.altitude, decDigit); Serial.print(",");
//   Serial.print(timer/1000); Serial.println();

// }

// void parseIMU(String data){
//   data = data.substring(VNRRG_LEN);
//   int asteriskIndex = data.indexOf('*');
//   data = data.substring(0, asteriskIndex);

//   imu.magX = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.magY = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.magZ = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.accelX = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.accelY = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.accelZ = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.gyroX = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.gyroY = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.gyroZ = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.temperature = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);
  
//   imu.pressure = data.toFloat(); 

// }

// void parseQT(String data) {
//     // Remove the "$VNRRG,09," part and focus on the quaternion data
//   data = data.substring(VNRRG_LEN); // Skip the first 10 characters "$VNRRG,09,"

//   // Find the asterisk (*) index to remove the checksum part
//   int asteriskIndex = data.indexOf('*');
//   if (asteriskIndex != -1) {
//     data = data.substring(0, asteriskIndex); // Remove the checksum
//   }

//   // Parse the quaternion components (w, x, y, z) from the comma-separated data
//   imu.qw = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);

//   imu.qx = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);

//   imu.qy = data.substring(0, data.indexOf(',')).toFloat();
//   data = data.substring(data.indexOf(',') + 1);

//   imu.qz = data.toFloat(); // The remaining part is the last quaternion value
// }
