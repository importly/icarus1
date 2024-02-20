#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <rocket_log.h>
#include <SD.h>
#include <SPI.h>

#define DEBUG 1
#define SEA_LEVEL_PRESSURE_HPA (1013.25)

#define ACCEL_SCALE_MODIFIER_2G 16384.0
#define ACCEL_SCALE_MODIFIER_4G 8192.0
#define ACCEL_SCALE_MODIFIER_8G 4096.0
#define ACCEL_SCALE_MODIFIER_16G 2048.0

#define GYRO_SCALE_MODIFIER_250DEG 131.0
#define GYRO_SCALE_MODIFIER_500DEG 65.5
#define GYRO_SCALE_MODIFIER_1000DEG 32.8
#define GYRO_SCALE_MODIFIER_2000DEG 16.4

#define ACCEL_RANGE_2G 0x00
#define ACCEL_RANGE_4G 0x08
#define ACCEL_RANGE_8G 0x10
#define ACCEL_RANGE_16G 0x18

#define GYRO_RANGE_250DEG 0x00
#define GYRO_RANGE_500DEG 0x08
#define GYRO_RANGE_1000DEG 0x10
#define GYRO_RANGE_2000DEG 0x18


// I2C addresses
const uint8_t MPU6050_ADDRESS = 0x68;
const uint8_t BMP390_ADDRESS = 0x77;

// Devices
Adafruit_BMP3XX bmp;

Servo pulley;
Servo steers[4];  // create an array of servo objects to control 4 servos
int calbration[4] = { 5, 0, -7, -6 };

void setup() {
    Serial.begin(115200);

    // MPU 6050
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x6B); // Power Management 1 register
    Wire.write(0);    // Set to 0 to wake up the MPU6050
    Wire.endTransmission(true);

    // Configure Accelerometer sensitivity (±16g)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1C);
    Wire.write(ACCEL_RANGE_16G); // Accel full scale range ±16g
    Wire.endTransmission(true);

    // Configure Gyroscope sensitivity (±250 deg/s)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1B);
    Wire.write(GYRO_RANGE_250DEG); // Gyro full scale range ±250 deg/s
    Wire.endTransmission(true);

    // Set Low Pass Filter (Bandwidth 184Hz)
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(0x1A);
    Wire.write(0x01); // Low Pass Filter setting
    Wire.endTransmission(true);

    // BMP 390
    if (bmp.begin_I2C(BMP390_ADDRESS, &Wire1))
        Serial.println("Found BMP390");
    else while (true) { Serial.println("Could not find a valid BMP390 sensor, check wiring!"); }


    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);

    if (DEBUG) Serial.println("AccelRateX,AccelRateY,AccelRateZ,GyroRateX,GyroRateY,GyroRateZ,Altitude");
}

int16_t readMPU6050(int address, uint8_t registerAddress) {
    Wire.beginTransmission(address);
    Wire.write(registerAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(address, 2);
    int16_t data = Wire.read() << 8 | Wire.read();
    return data;
}

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float accelRateX, accelRateY, accelRateZ, gyroRateX, gyroRateY, gyroRateZ, altitude;

void loop() {
    // Read accelerometer data
    accX = readMPU6050(MPU6050_ADDRESS, 0x3B);
    accY = readMPU6050(MPU6050_ADDRESS, 0x3D);
    accZ = readMPU6050(MPU6050_ADDRESS, 0x3F);

    // Read gyroscope data
    gyroX = readMPU6050(MPU6050_ADDRESS, 0x43);
    gyroY = readMPU6050(MPU6050_ADDRESS, 0x45);
    gyroZ = readMPU6050(MPU6050_ADDRESS, 0x47);

    // Read pressure data
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    // Convert accelerometer data to g's
    accelRateX = accX / ACCEL_SCALE_MODIFIER_16G; // For ±16g sensitivity
    accelRateY = accY / ACCEL_SCALE_MODIFIER_16G;
    accelRateZ = accZ / ACCEL_SCALE_MODIFIER_16G;

    // Convert gyro data to degrees per second
    gyroRateX = gyroX / GYRO_SCALE_MODIFIER_250DEG; // For ±250 deg/s sensitivity
    gyroRateY = gyroY / GYRO_SCALE_MODIFIER_250DEG;
    gyroRateZ = gyroZ / GYRO_SCALE_MODIFIER_250DEG;

    if (DEBUG) {
        Serial.print(accelRateX);
        Serial.print(",");
        Serial.print(accelRateY);
        Serial.print(",");
        Serial.println(accelRateZ);
//        Serial.print(",");
//        Serial.print(gyroRateX);
//        Serial.print(",");
//        Serial.print(gyroRateY);
//        Serial.print(",");
//        Serial.print(gyroRateZ);
//        Serial.print(",");
//        Serial.println(altitude);
    }
    delay(50);
}