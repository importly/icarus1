#ifndef ROCKET_LOG_H
#define ROCKET_LOG_H

#include <stdint.h>

struct rocket_log {
    uint32_t counter;       // Log entry counter
    uint32_t record_offset; // Offset in the log file or storage
    uint32_t time;          // Timestamp or elapsed time since start in milliseconds
    uint32_t dummy;         // Placeholder for alignment or future use

    // Accelerometer data
    float accelX;
    float accelY;
    float accelZ;

    // Gyroscope data
    float gyroRateX;
    float gyroRateY;
    float gyroRateZ;

    // BMP390 sensor data
    float temperature; // Temperature in degrees Celsius
    float pressure;    // Pressure in hPa
    float altitude;    // Calculated altitude in meters
};

#endif
