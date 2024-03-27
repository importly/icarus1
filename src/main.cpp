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
const int BUZZER_PIN = 36;
const int PULLEY_PIN = 29;
const int SX1_PIN = 3;
const int SX2_PIN = 7;
const int SY1_PIN = 10;
const int SY2_PIN = 25;

Kalman kalman_gyro_X;
Kalman kalman_gyro_Y;
Kalman kalman_gyro_Z;

Kalman kalman_acc_X;
Kalman kalman_acc_Y;
Kalman kalman_acc_Z;

bool SILENT = true;
bool DEBUG = true;

float altitude=0.0;

bool calibrate = false;
float altitude_c = 0.0;

int i = 0;

double kal_pitch;
double kal_roll;
double acc_pitch;
double acc_roll;

int servo_translate(int pos) {
    return pos + 90;
}

enum RocketState {
    IDLE,
    BOOST,
    COAST,
    APOGEE,
    LANDING,
};

RocketState rocket_state = IDLE;
unsigned long rocket_start_t = 0;
unsigned long rocket_boost_t = 0;
unsigned long rocket_coast_t = 0;

int servo_position_x = 0;
int servo_position_y = 0;

float target_altitude = 850;


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

    sx1.attach(SX1_PIN);
    sx2.attach(SX2_PIN);
    sy1.attach(SY1_PIN);
    sy2.attach(SY2_PIN);

    pinMode(BUZZER_PIN, OUTPUT); // set the buzzer pin as output

    // play two beeps to signal the system is ready
    if (!SILENT) {
        digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on
        delay(100); // wait for 200ms
        digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off
        delay(150); // wait for 300ms
        digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on again
        delay(100); // wait for 200ms
        digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off
    }
    if (DEBUG) Serial.println("AccelRateX,AccelRateY,AccelRateZ,GyroRateX,GyroRateY,GyroRateZ,Altitude");
}

void loop() {
    // Read pressure data
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    Vector acc = mpu.readNormalizeAccel();
    Vector gyr = mpu.readNormalizeGyro();

    // altitude in meters
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);

    altitude -= altitude_c;

    if (!calibrate) {
        altitude_c = altitude;
        calibrate = true;
    }

    // altitude to feet
    altitude *= 3.28084;


    switch (rocket_state) {
        case IDLE:
            if ((millis() - rocket_start_t > 5000) && (acc.ZAxis > 50 || acc.YAxis > 50 || acc.XAxis > 50)) { //TODO check acc
                rocket_state = BOOST;
                rocket_boost_t = millis();
            }
            break;
        case BOOST:
            if (millis() - rocket_boost_t > 500 && acc.ZAxis < 0) {
                rocket_state = COAST;
                rocket_coast_t = millis();
            }
            break;
        case COAST:
            acc_pitch = -(atan2(acc.XAxis, sqrt(acc.YAxis * acc.YAxis + acc.ZAxis * acc.ZAxis)) * 180.0) / M_PI;
            acc_roll  = (atan2(acc.YAxis, acc.ZAxis) * 180.0) / M_PI - 90;

            kal_pitch = kalman_acc_X.update(acc_pitch, gyr.YAxis);
            //    kal_roll = kalman_acc_Y.update(acc_roll, gyr.XAxis);

            servo_position_x = -acc_pitch;
            servo_position_y = acc_roll;

            // Assign the converted integer to all the servos
            sx1.write(servo_translate(servo_position_x));
            sx2.write(servo_translate(-servo_position_x));
            sy1.write(servo_translate(servo_position_y));
            sy2.write(servo_translate(-servo_position_y));

            if (millis() - rocket_coast_t  > 2000 && altitude > target_altitude ) {
                rocket_state = APOGEE;
            }
            break;
        case APOGEE:
            break;
        case LANDING:
            break;
    }

    if (DEBUG) {
        char buffer[100];  // Buffer to hold the formatted string
        sprintf(buffer, "%4.2f %4.2f %4.2f %4.2f", acc.XAxis, acc.YAxis,acc.ZAxis, kal_pitch);
        Serial.println(buffer);
    }
    delay(50);
}