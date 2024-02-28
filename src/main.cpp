#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <rocket_log.h>
#include <MPU6050.h>

#include <SD.h>
#include <SPI.h>

#include "../lib/KalmanFilter-master/Kalman.h"

#define DEBUG 1
#define SEA_LEVEL_PRESSURE_HPA (1013.25)

// I2C addresses
const uint8_t BMP390_ADDRESS = 0x77;

// Devices
Adafruit_BMP3XX bmp;
MPU6050 mpu;

Servo pulley;

Servo sx1;
Servo sx2;
Servo sy1;
Servo sy2;

// Beeper
int BUZZER_PIN = 36;
int PULLEY_PIN = 29;

Kalman kalman_gyro_X;
Kalman kalman_gyro_Y;
Kalman kalman_gyro_Z;

Kalman kalman_acc_X;
Kalman kalman_acc_Y;
Kalman kalman_acc_Z;

void setup() {
    Serial.begin(115200);

    // BMP 390
    if (bmp.begin_I2C(BMP390_ADDRESS, &Wire1))
        Serial.println("Found BMP390");
    else while (true) { Serial.println("Could not find a valid BMP390 sensor, check wiring!"); }

    // MPU6050
    if (mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_16G, 0x68))
        Serial.println("Found MPU6050");
    else while (true) { Serial.println("Could not find a valid MPU6050 sensor, check wiring!"); }


    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_32X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_1);
    bmp.setOutputDataRate(BMP3_ODR_200_HZ);

    mpu.calibrateGyro(100);
    Serial.println(mpu.getAccelOffsetX());
    Serial.println(mpu.getAccelOffsetY());
    Serial.println(mpu.getAccelOffsetZ());

    for (int i = 0; i < 100; i++) {
        bmp.performReading();
    }

    sx1.attach(3);
    sx2.attach(7);
    sy1.attach(10);
    sy2.attach(25);

    pinMode(BUZZER_PIN, OUTPUT); // set the buzzer pin as output

//    // play two beeps to signal the system is ready
//    digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on
//    delay(100); // wait for 200ms
//    digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off
//    delay(150); // wait for 300ms
//    digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on again
//    delay(100); // wait for 200ms
//    digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off

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

float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, altitude;

bool calibrate = false;
float altitude_c = 0.0;

int i = 0;

double kalPitch;
double kalRoll;
double accPitch;
double accRoll;

int servo_translate(int pos) {
    return pos + 90;
}


void loop() {
    // Read pressure data
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }
    Vector acc = mpu.readNormalizeAccel();
    Vector gyr = mpu.readNormalizeGyro();


    accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis*acc.YAxis + acc.ZAxis*acc.ZAxis))*180.0)/M_PI;
    accRoll  = (atan2(acc.YAxis, acc.ZAxis)*180.0)/M_PI - 90;

    //dt is the time between readings in seconds
    //TODO: Adust as necessary
    float dt = 0.05;
    kalPitch = kalman_acc_X.getAngle(accPitch, gyr.YAxis, dt);
    kalRoll = kalman_acc_Y.getAngle(accRoll, gyr.XAxis, dt);

    // altitude in meters
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);

    altitude -= altitude_c;

    if (!calibrate) {
        altitude_c = altitude;
        calibrate = true;
    }

    int servoPositionx = -kalPitch;
    int servoPositiony = kalRoll;

    // Assign the converted integer to all the servos
    sx1.write(servo_translate(servoPositionx));
    sx2.write(servo_translate(-servoPositionx));
    sy1.write(servo_translate(servoPositiony));
    sy2.write(servo_translate(-servoPositiony));

    if (DEBUG) {
        char buffer[100];  // Buffer to hold the formatted string
        sprintf(buffer, "%4.2f %4.2f %4.2f", acc.XAxis, acc.YAxis,acc.ZAxis);
        Serial.println(buffer);
    }
    delay(50);
}