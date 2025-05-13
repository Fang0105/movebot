#ifndef __ARM_H__
#define __ARM_H__

#include <vector>
#include <string>
#include "utils.h"

class Arm{
    public:
        int joint_number;
        int rod_number;
        // std::vector<int> joint_angles;
        std::vector<Rectangle> rods;
        std::vector<std::string> axis;
        std::vector<std::pair<int, int>> joint_angles_range;

        Arm() = default;
        Arm(int joint_number, int rod_number);

        void initialize(int joint_number, int rod_number);
        void printInfo();

        void calculatePosture(Configuration &config);

        void printPosture(Configuration &config);

        bool collisionDetection(Configuration &config, std::vector<Rectangle> &obstacles);
};

#endif