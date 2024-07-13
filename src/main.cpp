#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <rocket_log.h>
#include <MPU6050.h>
#include "Kalman.h"
#include "Vec3d.h"
#include <SD.h>
#include <SPI.h>


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
const int SY1_PIN = 3;
const int SY2_PIN = 7;

const int SX1_PIN = 10;
const int SX2_PIN = 25;

Kalman kalman_gyro_X;
Kalman kalman_gyro_Y;
Kalman kalman_gyro_Z;

Kalman kalman_acc_X;
Kalman kalman_acc_Y;
Kalman kalman_acc_Z;

Kalman kalman_vel_X;
Kalman kalman_vel_Y;
Kalman kalman_vel_Z;

bool SILENT = false;
bool DEBUG = true;

float altitude = 0.0;

bool calibrate = false;
float altitude_calibration = 0.0;

int i = 0;

double acc_pitch;
double acc_roll;

int servo_translate(double pos) {
    return (int) std::clamp(pos, -90.0, 90.0) + 90;
}

enum RocketState {
    IDLE,
    BOOST,
    COAST,
    APOGEE,
    LANDING,
};

RocketState rocket_state = RocketState::IDLE;

unsigned long rocket_start_t = 0;
unsigned long rocket_boost_t = 0;
unsigned long rocket_coast_t = 0;

double servo_position_x = 0;
double servo_position_y = 0;

float target_altitude = 820.0 / 3.28084;

double kal_pitch;
double kal_roll;

float dt = 0.05;
float dt_kal = 0.4

Vector acc_sensor_data;
Vector gyr_sensor_data;

Vec3d acc = Vec3d(0, 0, 0);
Vec3d gyr = Vec3d(0, 0, 0);
Vec3d vel = Vec3d(0, 0, 0);
Vec3d last_acc = Vec3d(0, 0, 0);
Vec3d kal_velocity = Vec3d(0, 0, 0);

double adjustment_angle = 0.0;

double m = 651.0 / 1000.0;

float k = 0.5 * 1.223 * 0.477 * 0.00342;

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

// x = -1 to 1, y= -1 to 1 z = 0
void vector_fins(Vec3d angleVec, double pitch, double roll) {
    servo_position_x = pitch + angleVec.x;
    servo_position_y = roll + angleVec.y;

    // Assign the converted integer to all the servos
    sx1.write(servo_translate(servo_position_x));
    sx2.write(servo_translate(-servo_position_x));
    sy1.write(servo_translate(servo_position_y));
    sy2.write(servo_translate(-servo_position_y));
}


void loop() {
    // Read pressure data
    if (!bmp.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
    }

    acc_sensor_data = mpu.readNormalizeAccel();
    gyr_sensor_data = mpu.readNormalizeGyro();

    acc = Vec3d((double) acc_sensor_data.XAxis, (double) acc_sensor_data.YAxis, (double) acc_sensor_data.ZAxis);
    gyr = Vec3d((double) gyr_sensor_data.XAxis, (double) gyr_sensor_data.YAxis, (double) gyr_sensor_data.ZAxis);

    // altitude in meters
    altitude = bmp.readAltitude(SEA_LEVEL_PRESSURE_HPA);

    altitude -= altitude_calibration;

    if (!calibrate) {
        altitude_calibration = altitude;
        calibrate = true;
    }

//    // altitude to feet
//    altitude *= 3.28084;

    vel.x += 0.5 * (acc.x + last_acc.x) * dt;
    vel.y += 0.5 * (acc.y + last_acc.y) * dt;
    vel.z += 0.5 * (acc.z - 9.81 + last_acc.z) * dt;

    kal_velocity = Vec3d(kalman_vel_X.getAngle(vel.x, (acc - last_acc).magnitude(), dt_kal),
                         kalman_vel_Y.getAngle(vel.y, (acc - last_acc).magnitude(), dt_kal),
                         kalman_vel_Z.getAngle(vel.z, (acc - last_acc).magnitude(), dt_kal));


    acc_pitch = -degrees(atan2(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)));
    acc_roll = -degrees(atan2(acc.y, acc.z)) + 90;



    //dt is the time between readings in seconds
    //TODO: Adust as necessary
    kal_pitch = kalman_acc_X.getAngle(acc_pitch, gyr.y,dt_kal);
    kal_roll = kalman_acc_Y.getAngle(acc_roll, gyr.y, dt_kal);

    // Assuming h_target, h_current, k, v (velocity), m (mass), and g (gravitational constant) are already defined

    vel.z = vel.z > 1.0 ? vel.z : 3.0;

    adjustment_angle = degrees(
            acos((target_altitude - altitude) * (2.0 * k / (m * log(1 + k * vel.z * vel.z / (m * 9.80665))))));

    if (std::isnan(adjustment_angle))
        adjustment_angle = 0.0;


    switch (rocket_state) {
        case IDLE:
            if ((millis() - rocket_start_t > 5000) && (abs(acc.z) > 15.0)) { //TODO check acc
                rocket_state = BOOST;
                rocket_boost_t = millis();
            }
            break;
        case BOOST:

            sx1.write(servo_translate(0));
            sx2.write(servo_translate(0));
            sy1.write(servo_translate(0));
            sy2.write(servo_translate(0));

            if (millis() - rocket_boost_t > 1000 && acc.z < 10) {
                rocket_state = COAST;
                rocket_coast_t = millis();
            }
            break;
        case COAST:
//            vector_fins(Vec3d(adjustment_angle, 0, 0), acc_pitch, acc_roll);

            servo_position_x = kal_pitch + adjustment_angle;
            servo_position_y = kal_roll + 0;

            // Assign the converted integer to all the servos
            sx1.write(servo_translate(servo_position_x));
            sx2.write(servo_translate(-servo_position_x));
            sy1.write(servo_translate(servo_position_y));
            sy2.write(servo_translate(-servo_position_y));

//            char buffer[100];  // Buffer to hold the formatted string
//            sprintf(buffer, "%4.2f %4.2f %4.2f %4.2f %4.2f %4.2f %4.2f ", target_altitude, altitude, m, k, vel.z,
//                    (target_altitude - altitude) * ((2.0 * k) / (m * (log(1 + (k * vel.z * vel.z) / (m * 9.80665))))),
//                    adjustment_angle);
//            Serial.println(buffer);

            if (millis() - rocket_coast_t > 2000 && altitude > target_altitude-15) {
                rocket_state = APOGEE;
            }
            break;
        case APOGEE:
            if (altitude < target_altitude-30) {
                rocket_state = LANDING;
            }
            break;
        case LANDING:
            sx1.write(servo_translate(90));
            sx2.write(servo_translate(90));
            sy1.write(servo_translate(90));
            sy2.write(servo_translate(90));
            if (!SILENT) {
                digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on
                delay(100); // wait for 200ms
                digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off
                delay(150); // wait for 300ms
                digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on again
                delay(100); // wait for 200ms
                digitalWrite(BUZZER_PIN, LOW); // turn the buzzer off
            }
            break;
    }

    if (millis() - rocket_start_t > 100000){
        digitalWrite(BUZZER_PIN, HIGH); // turn the buzzer on
    }
    last_acc = acc;

    if (DEBUG) {
//        char buffer[100];  // Buffer to hold the formatted string
//        sprintf(buffer, "%4.2f %4.2f %4.2f %4.2f", acc.x, acc.y, acc.z, kal_pitch);
//        Serial.println(buffer);
    }
    delay(50);
}

