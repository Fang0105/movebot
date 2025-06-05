#ifndef __UTILS_H__
#define __UTILS_H__

#include <vector>
#include <iostream>
#include <array>
#include <cmath>
#include <arm_neon.h>
#include <chrono>

#define RED "\e[0;31m"
#define NONE "\e[0m"
#define CYAN "\e[0;36m"
#define GREEN "\e[0;32m"
#define BOLD "\e[1m"
#define PURPLE "\e[0;35m"

#define PI 3.1415926
#define RAD(X) (static_cast<float>(X) * PI / 180.0)
#define DEG(X) ((X) * 180.0 / PI)

#define CHECKINGPOINTSNUMBER 16

struct Configuration{
    int joint_number;
    std::vector<int> joint_angles;
    Configuration() : joint_number(0){}
    Configuration(int joint_number) : joint_number(joint_number), joint_angles(joint_number, 0) {}
    Configuration(int joint_number, std::vector<int>joint_angles) : joint_number(joint_number), joint_angles(joint_angles) {}
    friend std::ostream &operator<<(std::ostream &os, const Configuration &config) {
        os << "{";
        for (int i = 0; i < config.joint_number; ++i) {
            if(i!=0){
                os << ", ";
            }
            os << config.joint_angles[i];
        }
        os << "}\n";
        return os;
    }

    bool operator==(const Configuration &other) const {
        if (joint_number != other.joint_number) return false;
        for (int i = 0; i < joint_number; ++i) {
            if (joint_angles[i] != other.joint_angles[i]) return false;
        }
        return true;
    }

    bool operator!=(const Configuration &other) const {
        return !(*this == other);
    }

    bool operator<(const Configuration &other) const {
        if (joint_number != other.joint_number) return joint_number < other.joint_number;
        for (int i = 0; i < joint_number; ++i) {
            if (joint_angles[i] != other.joint_angles[i]) return joint_angles[i] < other.joint_angles[i];
        }
        return false;
    }

    Configuration operator-(const Configuration &other) const {
        Configuration result(joint_number);
        for (int i = 0; i < joint_number; ++i) {
            result.joint_angles[i] = joint_angles[i] - other.joint_angles[i];
        }
        return result;
    }
};

struct Vec3{
    double x;
    double y;
    double z;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3() : x(0), y(0), z(0) {}

    Vec3 operator+(const Vec3 &other) const {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }
    Vec3 operator-(const Vec3 &other) const {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    operator struct Point() const;

    Vec3 operator*(double scalar) const {
        return Vec3(x * scalar, y * scalar, z * scalar);
    }

    friend std::ostream &operator<<(std::ostream &os, const Vec3 &v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

struct Point{
    float x;
    float y;
    float z;
    Point(float x, float y, float z) : x(x), y(y), z(z) {}
    Point() : x(0), y(0), z(0) {}

    operator Vec3() const {
        return Vec3(x, y, z);
    }

    friend std::ostream &operator<<(std::ostream &os, const Point &point) {
        os << "(" << point.x << ", " << point.y << ", " << point.z << ")";
        return os;
    }
};

inline Vec3::operator Point() const {
    return Point(x, y, z);
}

inline Vec3 normalize(Vec3 vec){
    float x, y, z;
    float length = std::sqrt(vec.x * vec.x + vec.y * vec.y + vec.z * vec.z);
    x = vec.x / length;
    y = vec.y / length;
    z = vec.z / length;
    return Vec3(x, y, z);
}

inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct Rectangle{
    Point bottom_center;
    Point top_center;
    Point center;
    float width; // y-direction
    float length; // x-direction
    float height;  // z-direction
    Vec3 side_vector;
    Rectangle() : width(0), length(0), height(0), side_vector(0.0, -1.0, 0.0) {}
    Rectangle(float length, float width, float height) : width(width), length(length), height(height), side_vector(0.0, -1.0, 0.0){}
    // friend std::ostream &operator<<(std::ostream &os, const Rectangle &rect) {
    //     os << CYAN << "Length = " << NONE << rect.length;
    //     os << CYAN << ", Width = " << NONE << rect.width;
    //     os << CYAN << ", Height = " << NONE << rect.height;
    //     return os;
    // }

    Vec3 up_vector() const {
        return normalize(static_cast<Vec3>(top_center) - static_cast<Vec3>(bottom_center));
    }

    Vec3 forward_vector() const {
        return normalize(static_cast<Vec3>(side_vector));
    }

    Vec3 right_vector() const {
        return normalize(cross(up_vector(), forward_vector()));
    }

    std::array<Vec3, 3> get_axes() const {
        return { forward_vector(), right_vector(), up_vector() };
    }
    
    std::array<float, 3> get_half_sizes() const {
        return { length / 2.0f, width / 2.0f, height / 2.0f };
    }

    friend std::ostream &operator<<(std::ostream &os, const Rectangle &rect) {
        os << CYAN << "Length = " << NONE << rect.length << std::endl;
        os << CYAN << ", Width = " << NONE << rect.width << std::endl;
        os << CYAN << ", Height = " << NONE << rect.height << std::endl;
        os << CYAN << ", Bottom Center = " << NONE << rect.bottom_center << std::endl;
        os << CYAN << ", Top Center = " << NONE << rect.top_center << std::endl;
        os << CYAN << ", Center = " << NONE << rect.center << std::endl;
        os << CYAN << ", Side Vector = " << NONE << rect.side_vector << std::endl;

        return os;
    }
};

struct Sphere{
    float radius;
    Point center;
    Sphere(float radius, Point center) : radius(radius), center(center) {}
    Sphere() : radius(1.0f), center(Point(0.0f, 0.0f, 0.0f)) {}

    friend std::ostream &operator<<(std::ostream &os, const Sphere &sphere){
        os << CYAN << "Radius = " << NONE << sphere.radius << std::endl;
        os << CYAN << "Center = " << NONE << sphere.center << std::endl;
        return os;
    }
};


inline float32x4_t degrees_to_radians(float32x4_t degrees) {
    const float32x4_t factor = vdupq_n_f32(3.14159265358979323846f / 180.0f);
    return vmulq_f32(degrees, factor);
}


// ------------------------ collision detection ------------------------

// 計算在某個 axis 上的投影範圍 [min, max]
inline void projectBoxOntoAxis(const Rectangle& box, const Vec3& axis, float& outMin, float& outMax) {
    std::array<Vec3, 3> axes = box.get_axes();
    std::array<float, 3> halfSizes = box.get_half_sizes();

    Vec3 center = static_cast<Vec3>(box.center);
    float projectionCenter = dot(center, axis);
    float projectionRadius =
        std::abs(dot(axes[0], axis)) * halfSizes[0] +
        std::abs(dot(axes[1], axis)) * halfSizes[1] +
        std::abs(dot(axes[2], axis)) * halfSizes[2];

    outMin = projectionCenter - projectionRadius;
    outMax = projectionCenter + projectionRadius;
}

inline bool overlap(float minA, float maxA, float minB, float maxB) {
    return !(maxA < minB || maxB < minA);
}

inline bool cuboidCuboidCollisionDetection(Rectangle& A, Rectangle& B) {
    auto axesA = A.get_axes();
    auto axesB = B.get_axes();

    std::vector<Vec3> testAxes;

    // 3 軸來自 A
    testAxes.insert(testAxes.end(), axesA.begin(), axesA.end());

    // 3 軸來自 B
    testAxes.insert(testAxes.end(), axesB.begin(), axesB.end());

    // 9 軸為 A×B
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vec3 crossAxis = cross(axesA[i], axesB[j]);
            if (crossAxis.x != 0 || crossAxis.y != 0 || crossAxis.z != 0) {
                testAxes.push_back(normalize(crossAxis)); // 要 normalize
            }
        }
    }

    // 檢查每一個 axis 上的投影是否有重疊
    for (const Vec3& axis : testAxes) {
        float minA, maxA, minB, maxB;
        projectBoxOntoAxis(A, axis, minA, maxA);
        projectBoxOntoAxis(B, axis, minB, maxB);
        if (!overlap(minA, maxA, minB, maxB)) {
            // 有一個軸不重疊，就表示沒有碰撞
            return false;
        }
    }

    return true; // 所有軸都重疊，表示有碰撞
}

inline auto get_elapsed_nanoseconds(const std::chrono::time_point<std::chrono::steady_clock> &start) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
}

struct validation_result {
    bool valid;
    long long FK_time;
    long long CC_time;

    validation_result() : valid(false), FK_time(0.0), CC_time(0.0) {};
    validation_result(bool valid, long long FK_time, long long CC_time)
        : valid(valid), FK_time(FK_time), CC_time(CC_time) {}
};

// ---------------------------------------------------------------------

#endif