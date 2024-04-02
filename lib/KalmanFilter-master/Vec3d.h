//
// Created by nlplocaladmin on 3/27/2024.
//

class Vec3d {
public:
    double x, y, z;

    explicit Vec3d(double x = 0, double y = 0, double z = 0);

    Vec3d operator+(const Vec3d& rhs) const;
    Vec3d& operator+=(const Vec3d& rhs);
    Vec3d operator-(const Vec3d& rhs) const;
    Vec3d& operator-=(const Vec3d& rhs);
    Vec3d operator*(double scalar) const;
    Vec3d& operator*=(double scalar);
    Vec3d operator/(double scalar) const;
    Vec3d& operator/=(double scalar);
    double dot(const Vec3d& rhs) const;
    Vec3d cross(const Vec3d& rhs) const;
    double magnitude() const;
    Vec3d normalize() const;
};
