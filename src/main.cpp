#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <rocket_log.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <SD.h>
#include <SPI.h>

#define DEBUG 1
#define SEA_LEVEL_PRESSURE_HPA (1013.25)

// I2C addresses
const uint8_t BMP390_ADDRESS = 0x77;

// Devices
Adafruit_BMP3XX bmp;
MPU6050 mpu;

Servo pulley;
Servo steers[4];  // create an array of servo objects to control 4 servos
int calbration[4] = { 5, 0, -7, -6 };

// Beeper
int BUZZER_PIN = 15;
int pulleyint = 29;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup() {
    Serial.begin(115200);

    // BMP 390
    if (bmp.begin_I2C(BMP390_ADDRESS, &Wire1))
        Serial.println("Found BMP390");
    else while (true) { Serial.println("Could not find a valid BMP390 sensor, check wiring!"); }

    // MPU6050
    if (mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_16G))
        Serial.println("Found MPU6050");
    else while (true) { Serial.println("Could not find a valid MPU6050 sensor, check wiring!"); }


    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);


    pinMode(BUZZER_PIN, OUTPUT); // set the buzzer pin as output

    // play two beeps to signal the system is ready
    digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on
    delay(200); // wait for 200ms
    digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off
    delay(300); // wait for 300ms
    digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on again
    delay(200); // wait for 200ms
    digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off


    mpu.calibrateGyro();

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

float accelRateX, accelRateY, accelRateZ, gyroRateX, gyroRateY, gyroRateZ, altitude;

bool calibrate = false;
float altitude_c = 0.0;

int i = 0 ;

void loop() {
    analogWrite(pulleyint, 255);

    // Read pressure data
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    Vector acc = mpu.readNormalizeAccel();
    Vector gyr = mpu.readNormalizeGyro();

    // Calculate Pitch & Roll from accelerometer (deg)
    accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
    accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI;

    // Kalman filter
    kalPitch = kalmanY.update(accPitch, gyr.YAxis);
    kalRoll = kalmanX.update(accRoll, gyr.XAxis);

    // altitude in meters
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);

    altitude -= altitude_c;

    if (!calibrate) {
        altitude_c = altitude;

        calibrate = true;
    }

    if (DEBUG) {

        Serial.print(accPitch);
        Serial.print(":");
        Serial.print(accRoll);
        Serial.print(":");
        Serial.print(kalPitch);
        Serial.print(":");
        Serial.print(kalRoll);
        Serial.print(":");
        Serial.print(acc.XAxis);
        Serial.print(":");
        Serial.print(acc.YAxis);
        Serial.print(":");
        Serial.print(acc.ZAxis);
        Serial.print(":");
        Serial.print(gyr.XAxis);
        Serial.print(":");
        Serial.print(gyr.YAxis);
        Serial.print(":");
        Serial.print(gyr.ZAxis);

        Serial.println();
//        Serial.print(accelRateX);
//        Serial.print(",");
//        Serial.print(accelRateY);
//        Serial.print(",");
//        Serial.println(accelRateZ);
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