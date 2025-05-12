#ifndef __UTILS_H__
#define __UTILS_H__

#include <vector>
#include <iostream>

#define RED "\e[0;31m"
#define NONE "\e[0m"
#define CYAN "\e[0;36m"
#define GREEN "\e[0;32m"
#define BOLD "\e[1m"
#define PURPLE "\e[0;35m"

#define PI 3.1415926
#define RAD(X) ((X) * PI / 180.0)
#define DEG(X) ((X) * 180.0 / PI)

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

struct Point{
    double x;
    double y;
    double z;
    Point(double x, double y, double z) : x(x), y(y), z(z) {}
    Point() : x(0), y(0), z(0) {}
};

struct Vec3{
    double x;
    double y;
    double z;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}
    Vec3() : x(0), y(0), z(0) {}
};

struct Rectangle{
    Point bottom_center;
    Point top_center;
    double width; // y-direction
    double length; // x-direction
    double height;  // z-direction
    Vec3 forward_direction;
    Rectangle() : width(0), length(0), height(0), forward_direction(0.0, 1.0, 0.0) {}
    Rectangle(double length, double width, double height) : width(width), length(length), height(height), forward_direction(0.0, 1.0, 0.0){}
    friend std::ostream &operator<<(std::ostream &os, const Rectangle &rect) {
        os << CYAN << "Length = " << NONE << rect.length;
        os << CYAN << ", Width = " << NONE << rect.width;
        os << CYAN << ", Height = " << NONE << rect.height;
        return os;
    }
};

#endif