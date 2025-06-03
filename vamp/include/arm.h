#ifndef __ARM_H__
#define __ARM_H__

#include <vector>
#include <string>
#include "utils.h"
#include <arm_neon.h>

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

        bool motionValidation(Configuration &A, Configuration &B, std::vector<Sphere> &obstacles, bool DEBUG = false);

        bool batchCollisionDetection(std::vector<Configuration> &configs, std::vector<Sphere> &obstacles, bool DEBUG = false);

        bool batchSphereCollisionDetection(float32x4_t v_top_center[3], float32x4_t v_bottom_center[3], int rod_id, std::vector<Sphere> &obstacles, bool DEBUG = false);
};

#endif