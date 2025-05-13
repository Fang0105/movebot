#include "arm.h"
#include "utils.h"
#include <iostream>
#include <sstream>
#include <cmath>
// #include <regex>

Arm::Arm(int joint_number, int rod_number) {
    this->joint_number = joint_number;
    this->rod_number = rod_number;
    
    // this->joint_angles.resize(joint_number, 0);
    this->rods.resize(rod_number);
    this->axis.resize(joint_number, "z");
    this->joint_angles_range.resize(joint_number, std::make_pair(0, 180));
    std::cout << "Arm initialized with " << joint_number << " joints and " << rod_number << " rods." << std::endl;
}

void Arm::initialize(int joint_number, int rod_number) {
    this->joint_number = joint_number;
    this->rod_number = rod_number;
    
    // this->joint_angles.resize(joint_number, 0);
    this->rods.resize(rod_number);
    this->axis.resize(joint_number, "z");
    this->joint_angles_range.resize(joint_number, std::make_pair(0, 180));
    std::cout << "Arm initialized with " << joint_number << " joints and " << rod_number << " rods." << std::endl;
}

size_t visibleLength(const std::string& s) {
    size_t count = 0;
    bool in_escape = false;

    for (size_t i = 0; i < s.length(); ++i) {
        if (!in_escape) {
            if (s[i] == '\033' && i + 1 < s.length() && s[i + 1] == '[') {
                in_escape = true;
                ++i; // skip the '['
            } else {
                ++count;
            }
        } else {
            if ((s[i] >= 'a' && s[i] <= 'z') || (s[i] >= 'A' && s[i] <= 'Z')) {
                in_escape = false; // end of escape sequence
            }
        }
    }
    return count;
}


void Arm::printInfo(){
    std::string outter_boarder = "#===============================================================#";
    std::string inner_boarder = "|---------------------------------------------------------------|";
    std::cout << std::endl;

    std::cout << outter_boarder << std::endl;
    int padding = (outter_boarder.length() - 17) / 2;
    std::cout << "|";
    std::cout << std::string(padding, ' ');
    std::cout << RED << BOLD << "Arm Information" << NONE;
    std::cout << std::string(padding, ' ');
    std::cout << "|";
    std::cout << std::endl;
    std::cout << outter_boarder << std::endl;

    std::stringstream ss;
    std::string str;
    ss << CYAN << BOLD << "Number of joints: " << NONE << joint_number;
    getline(ss, str);
    {
        int total_width = outter_boarder.length() - 2;
        int visible = visibleLength(str);
        int left_pad = (total_width - visible) / 2;
        int right_pad = total_width - visible - left_pad;
        std::cout << "|" << std::string(left_pad, ' ') << str << std::string(right_pad, ' ') << "|" << std::endl;
    }
    ss.clear();



    std::cout << inner_boarder << std::endl;
    for(int i = 0; i < joint_number; i++){
        ss << GREEN << "Joint " << i+1 << ":" << NONE;
        getline(ss, str);
        std::cout << "|" << str << std::string(outter_boarder.length() - visibleLength(str) - 2, ' ') << "|" << std::endl;
        ss.clear();

        ss << CYAN << "  Axis: " << NONE << axis[i];
        getline(ss, str);
        std::cout << "|" << str << std::string(outter_boarder.length() - visibleLength(str) - 2, ' ') << "|" << std::endl;
        ss.clear();

        ss << CYAN << "  Angle range: " << NONE << "[" << joint_angles_range[i].first << ", " << joint_angles_range[i].second << "]";
        getline(ss, str);
        std::cout << "|" << str << std::string(outter_boarder.length() - visibleLength(str) - 2, ' ') << "|" << std::endl;
        ss.clear();
    }
    std::cout << inner_boarder << std::endl;

    ss << CYAN << BOLD << "Number of rods: " << NONE << joint_number;
    getline(ss, str);
    {
        int total_width = outter_boarder.length() - 2;
        int visible = visibleLength(str);
        int left_pad = (total_width - visible) / 2;
        int right_pad = total_width - visible - left_pad;
        std::cout << "|" << std::string(left_pad, ' ') << str << std::string(right_pad, ' ') << "|" << std::endl;
    }
    ss.clear();
    
    std::cout << inner_boarder << std::endl;
    for(int i = 0; i < rod_number; i++){
        ss << GREEN << "Rod " << i+1 << ":" << NONE;
        getline(ss, str);
        std::cout << "|" << str << std::string(outter_boarder.length() - visibleLength(str) - 2, ' ') << "|" << std::endl;
        ss.clear();

        ss << "  " << rods[i];
        getline(ss, str);
        std::cout << "|" << str << std::string(outter_boarder.length() - visibleLength(str) - 2, ' ') << "|" << std::endl;
        ss.clear();
    }
    std::cout << outter_boarder << std::endl;
    std::cout << std::endl;
}

void Arm::calculatePosture(Configuration &config) {
    Point last_top_center(0, 0, 0);
    rods[0].bottom_center = last_top_center;
    rods[0].top_center = Point(0, 0, rods[0].height);
    rods[0].center = static_cast<Point>((static_cast<Vec3>(rods[0].bottom_center) + static_cast<Vec3>(rods[0].top_center)) * 0.5);


    int xy_rotation_angle = config.joint_angles[0];
    int z_rotation_angle;

    if(this->axis[0] == "x"){
        z_rotation_angle = config.joint_angles[1];
    }else{
        z_rotation_angle = 180 - config.joint_angles[1];
    }

    rods[0].side_vector = Vec3(sin(RAD(xy_rotation_angle)), -cos(RAD(xy_rotation_angle)), 0);

    
    
    for(int i=1; i<rod_number-1; i++){
        // std::cout << "i: " << i << std::endl;
        // std::cout << "axis: " << this->axis[i] << std::endl;
        // std::cout << "joint_angles: " << config.joint_angles[i] << std::endl;
        if(i!=1){
            if(this->axis[i] == "x"){
                z_rotation_angle = z_rotation_angle - (90 - config.joint_angles[i]);
            }else if(this->axis[i] == "-x"){
                z_rotation_angle = z_rotation_angle - (90 - (180 - config.joint_angles[i]));
            }
        }
        this->rods[i].bottom_center = this->rods[i-1].top_center;
        Vec3 bottom_center = static_cast<Vec3>(rods[i].bottom_center);
        Vec3 local_top_center = Vec3(this->rods[i].height * cos(RAD(z_rotation_angle)) * cos(RAD(xy_rotation_angle)), 
                                                     this->rods[i].height * cos(RAD(z_rotation_angle)) * sin(RAD(xy_rotation_angle)), 
                                                     this->rods[i].height * sin(RAD(z_rotation_angle)));
        this->rods[i].top_center = static_cast<Point>(bottom_center + local_top_center);
        this->rods[i].side_vector = this->rods[0].side_vector;
        this->rods[i].center = static_cast<Point>((static_cast<Vec3>(rods[i].bottom_center) + static_cast<Vec3>(rods[i].top_center)) * 0.5);
    }

    this->rods[rod_number-1].bottom_center = this->rods[rod_number-2].top_center;
    Vec3 bottom_center = static_cast<Vec3>(rods[rod_number-1].bottom_center);
    Vec3 direction = normalize(static_cast<Vec3>(rods[rod_number-2].top_center) - static_cast<Vec3>(rods[rod_number-2].bottom_center));

    Vec3 local_top_center = direction * this->rods[rod_number-1].height;
    this->rods[rod_number-1].top_center = static_cast<Point>(bottom_center + local_top_center);
    this->rods[rod_number-1].side_vector = this->rods[0].side_vector; // TODO: fix this
    rods[rod_number-1].center = static_cast<Point>((static_cast<Vec3>(rods[rod_number-1].bottom_center) + static_cast<Vec3>(rods[rod_number-1].top_center)) * 0.5);
}

void Arm::printPosture(Configuration &config) {
    calculatePosture(config);
    std::cout << "Posture:" << std::endl;
    for(int i=0; i<rod_number; i++){
        std::cout << "Rod " << i+1 << ": ";
        std::cout << rods[i].bottom_center << " -> " << rods[i].top_center;
        std::cout << " | Center: " << rods[i].center;
        std::cout << " | Side vector: " << rods[i].side_vector;
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

bool Arm::collisionDetection(Configuration &config, std::vector<Rectangle> &obstacles) {
    calculatePosture(config);
    for(auto &obstacle : obstacles){
        for(auto &rod : rods){
            if(cuboidCuboidCollisionDetection(rod, obstacle)){
                return true;
            }
        }
    }
    return false;
}