#include "arm.h"
#include "utils.h"
#include <iostream>
#include <sstream>
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
