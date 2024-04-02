//
// Created by nlplocaladmin on 3/27/2024.
//

#include "Vec3d.h"
#include <cmath>


Vec3d::Vec3d(double x, double y, double z) : x(x), y(y), z(z) {}

Vec3d Vec3d::operator+(const Vec3d& rhs) const {
    return Vec3d(x + rhs.x, y + rhs.y, z + rhs.z);
}

Vec3d& Vec3d::operator+=(const Vec3d& rhs) {
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}

Vec3d Vec3d::operator-(const Vec3d& rhs) const {
    return Vec3d(x - rhs.x, y - rhs.y, z - rhs.z);
}

Vec3d& Vec3d::operator-=(const Vec3d& rhs) {
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}

Vec3d Vec3d::operator*(double scalar) const {
    return Vec3d(x * scalar, y * scalar, z * scalar);
}

Vec3d& Vec3d::operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vec3d Vec3d::operator/(double scalar) const {
    return Vec3d(x / scalar, y / scalar, z / scalar);
}

Vec3d& Vec3d::operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
}

double Vec3d::dot(const Vec3d& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
}

Vec3d Vec3d::cross(const Vec3d& rhs) const {
    return Vec3d(y * rhs.z - z * rhs.y, z * rhs.x - x * rhs.z, x * rhs.y - y * rhs.x);
}

double Vec3d::magnitude() const {
    return std::sqrt(x * x + y * y + z * z);
}

Vec3d Vec3d::normalize() const {
    double mag = magnitude();
    return Vec3d(x / mag, y / mag, z / mag);
}