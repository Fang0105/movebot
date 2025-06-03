#include "arm.h"
#include "utils.h"
#include <iostream>
#include <sstream>
#include <cmath>
#include <arm_neon.h>
#include <sleef.h>
#include <algorithm>

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

inline Configuration step_forward(Configuration &base, Configuration &direction, int step) {
    Configuration result(5);
    result.joint_angles[0] = base.joint_angles[0] + std::round((double)(direction.joint_angles[0]) * step / CHECKINGPOINTSNUMBER);
    result.joint_angles[1] = base.joint_angles[1] + std::round((double)(direction.joint_angles[1]) * step / CHECKINGPOINTSNUMBER);
    result.joint_angles[2] = base.joint_angles[2] + std::round((double)(direction.joint_angles[2]) * step / CHECKINGPOINTSNUMBER);
    result.joint_angles[3] = base.joint_angles[3] + std::round((double)(direction.joint_angles[3]) * step / CHECKINGPOINTSNUMBER);
    result.joint_angles[4] = base.joint_angles[4] + std::round((double)(direction.joint_angles[4]) * step / CHECKINGPOINTSNUMBER);
    return result;
}

validation_result Arm::motionValidation(Configuration &A, Configuration &B, std::vector<Sphere> &obstacles, bool DEBUG) {
    Configuration diff = B - A;
    std::vector<Configuration> configs(4);
    validation_result result, temp;

    // 1~4
    configs[0] = step_forward(A, diff, 1);
    configs[1] = step_forward(A, diff, 2);
    configs[2] = step_forward(A, diff, 3);
    configs[3] = step_forward(A, diff, 4);
    result = batchCollisionDetection(configs, obstacles, DEBUG);
    if(!result.valid){
        return result;
    }
    

    // 5~8
    configs[0] = step_forward(A, diff, 5);
    configs[1] = step_forward(A, diff, 6);
    configs[2] = step_forward(A, diff, 7);
    configs[3] = step_forward(A, diff, 8);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    // 9~12
    configs[0] = step_forward(A, diff, 9);
    configs[1] = step_forward(A, diff, 10);
    configs[2] = step_forward(A, diff, 11);
    configs[3] = step_forward(A, diff, 12);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    // 13~16
    configs[0] = step_forward(A, diff, 13);
    configs[1] = step_forward(A, diff, 14);
    configs[2] = step_forward(A, diff, 15);
    configs[3] = step_forward(A, diff, 16);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    // 17~20
    configs[0] = step_forward(A, diff, 17);
    configs[1] = step_forward(A, diff, 18);
    configs[2] = step_forward(A, diff, 19);
    configs[3] = step_forward(A, diff, 20);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    // 21~24
    configs[0] = step_forward(A, diff, 21);
    configs[1] = step_forward(A, diff, 22);
    configs[2] = step_forward(A, diff, 23);
    configs[3] = step_forward(A, diff, 24);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    // 25~28
    configs[0] = step_forward(A, diff, 25);
    configs[1] = step_forward(A, diff, 26);
    configs[2] = step_forward(A, diff, 27);
    configs[3] = step_forward(A, diff, 28);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    // 29~32
    configs[0] = step_forward(A, diff, 29);
    configs[1] = step_forward(A, diff, 30);
    configs[2] = step_forward(A, diff, 31);
    configs[3] = step_forward(A, diff, 32);
    temp = batchCollisionDetection(configs, obstacles, DEBUG);
    result.FK_time += temp.FK_time;
    result.CC_time += temp.CC_time;
    result.valid = temp.valid;
    if(!result.valid){
        return result;
    }

    return result;
}

bool Arm::batchSphereCollisionDetection(float32x4_t v_top_center[3], float32x4_t v_bottom_center[3], int rod_id, std::vector<Sphere> &obstacles, bool DEBUG) {
    std::vector<float>ball_numbers = {1.0f, 3.0f, 5.0f, 7.0f};

    bool valid = false;
    float32x4_t unit[3];
    for(float &n:ball_numbers){
        float32x4_t v_2n = vdupq_n_f32(2.0f * n);
        unit[0] = vdivq_f32(vsubq_f32(v_top_center[0], v_bottom_center[0]), v_2n);
        unit[1] = vdivq_f32(vsubq_f32(v_top_center[1], v_bottom_center[1]), v_2n);
        unit[2] = vdivq_f32(vsubq_f32(v_top_center[2], v_bottom_center[2]), v_2n);
        float r = std::max({rods[rod_id].height / (2.0f * n), rods[rod_id].width / 2.0f, rods[rod_id].length / 2.0f});

        bool collision_detected = false;
        for(int i=1;i<=n;i++){
            if(collision_detected){
                break;
            }
            float32x4_t v_center[3];
            v_center[0] = vaddq_f32(v_bottom_center[0], vmulq_n_f32(unit[0], 2.0f * ((float)i) - 1.0f));
            v_center[1] = vaddq_f32(v_bottom_center[1], vmulq_n_f32(unit[1], 2.0f * ((float)i) - 1.0f));
            v_center[2] = vaddq_f32(v_bottom_center[2], vmulq_n_f32(unit[2], 2.0f * ((float)i) - 1.0f));
            
            for(Sphere &obs: obstacles){
                float32x4_t v_obs_center[3] = {vdupq_n_f32(obs.center.x), vdupq_n_f32(obs.center.y), vdupq_n_f32(obs.center.z)};
                float32x4_t v_diff[3];
                v_diff[0] = vsubq_f32(v_center[0], v_obs_center[0]);
                v_diff[1] = vsubq_f32(v_center[1], v_obs_center[1]);
                v_diff[2] = vsubq_f32(v_center[2], v_obs_center[2]);

                float32x4_t v_center_distance_sq = vaddq_f32(vaddq_f32(vmulq_f32(v_diff[0], v_diff[0]), vmulq_f32(v_diff[1], v_diff[1])), vmulq_f32(v_diff[2], v_diff[2]));
                float32x4_t v_radius_sum_sq = vmulq_f32(vdupq_n_f32(r + obs.radius), vdupq_n_f32(r + obs.radius));

                uint32_t result[4];
                vst1q_u32(result, vcltq_f32(v_center_distance_sq, v_radius_sum_sq)); 
                
                if(result[0] || result[1] || result[2] || result[3]){
                    if(DEBUG){
                        std::cout << "Collision detected between rod " << rod_id << " and sphere at (" << obs.center.x << ", " << obs.center.y << ", " << obs.center.z << ") with radius " << obs.radius << std::endl;
                    }
                    collision_detected = true;
                    break;
                }

            }
        }
        if(!collision_detected){
            valid = true;
            if(DEBUG){
                std::cout << "No collision detected for rod " << rod_id << " with " << n << " balls." << std::endl;
            }
            break;
        }
    }

    return valid;
}

validation_result Arm::batchCollisionDetection(std::vector<Configuration> &configs, std::vector<Sphere> &obstacles, bool DEBUG) {

    validation_result result;
    Rectangle posture[5];


    float angle0[4] = {(float)configs[0].joint_angles[0], (float)configs[1].joint_angles[0], (float)configs[2].joint_angles[0], (float)configs[3].joint_angles[0]};
    float angle1[4] = {(float)configs[0].joint_angles[1], (float)configs[1].joint_angles[1], (float)configs[2].joint_angles[1], (float)configs[3].joint_angles[1]};
    float angle2[4] = {(float)configs[0].joint_angles[2], (float)configs[1].joint_angles[2], (float)configs[2].joint_angles[2], (float)configs[3].joint_angles[2]};
    float angle3[4] = {(float)configs[0].joint_angles[3], (float)configs[1].joint_angles[3], (float)configs[2].joint_angles[3], (float)configs[3].joint_angles[3]};
    // float angle4[4] = {(float)configs[0].joint_angles[4], (float)configs[1].joint_angles[4], (float)configs[2].joint_angles[4], (float)configs[3].joint_angles[4]};
    // float32x4_t v_angle0 = vld1q_f32(angle0);
    float32x4_t v_angle1 = vld1q_f32(angle1);
    float32x4_t v_angle2 = vld1q_f32(angle2);
    float32x4_t v_angle3 = vld1q_f32(angle3);
    // float32x4_t v_angle4 = vld1q_f32(angle4);
    float32x4_t v_180 = vdupq_n_f32(180.0f);

    // z -x x -x y, 0-180
    auto FK_start_time = std::chrono::steady_clock::now();
    float top_center[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {rods[0].height, rods[0].height, rods[0].height, rods[0].height}}; // {{x},{y},{z}}
    float bottom_center[3][4] = {{0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}}; // {{x},{y},{z}}
    float32x4_t v_top_center[3] = {vld1q_f32(top_center[0]), vld1q_f32(top_center[1]), vld1q_f32(top_center[2])};
    float32x4_t v_bottom_center[3] = {vld1q_f32(bottom_center[0]), vld1q_f32(bottom_center[1]), vld1q_f32(bottom_center[2])};

    float xy_rotation_angle[4];
    std::copy(std::begin(angle0), std::end(angle0), xy_rotation_angle);
    float32x4_t v_xy_rotation_angle = vld1q_f32(xy_rotation_angle);

    // float z_rotation_angle[4];
    float32x4_t v_z_rotation_angle = vsubq_f32(v_180, v_angle1);
    
    if(DEBUG){
        vst1q_f32(top_center[0], v_top_center[0]);
        vst1q_f32(top_center[1], v_top_center[1]);
        vst1q_f32(top_center[2], v_top_center[2]);
        vst1q_f32(bottom_center[0], v_bottom_center[0]);
        vst1q_f32(bottom_center[1], v_bottom_center[1]);
        vst1q_f32(bottom_center[2], v_bottom_center[2]);
        posture[0].bottom_center.x = bottom_center[0][0];
        posture[0].bottom_center.y = bottom_center[1][0];
        posture[0].bottom_center.z = bottom_center[2][0];
        posture[0].top_center.x = top_center[0][0];
        posture[0].top_center.y = top_center[1][0];
        posture[0].top_center.z = top_center[2][0];
        posture[0].length = rods[0].length;
        posture[0].width = rods[0].width;
        posture[0].height = rods[0].height;
    }
    result.FK_time = get_elapsed_nanoseconds(FK_start_time);
    // first rod
    auto CC_start_time = std::chrono::steady_clock::now();
    result.valid = batchSphereCollisionDetection(v_top_center, v_bottom_center, 0, obstacles, DEBUG);
    result.CC_time += get_elapsed_nanoseconds(CC_start_time);
    if(result.valid == false){
        if(DEBUG){
            std::cout << "Collision detected for 1st rod." << std::endl;
        }
        return result;
    }
    
    // second rod, -x
    FK_start_time = std::chrono::steady_clock::now();
    v_bottom_center[0] = v_top_center[0];
    v_bottom_center[1] = v_top_center[1];
    v_bottom_center[2] = v_top_center[2];
    float32x4_t v_local_top_center[3] = {
        vmulq_f32(vdupq_n_f32(rods[1].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_cosf4_u10(degrees_to_radians(v_xy_rotation_angle)))),
        vmulq_f32(vdupq_n_f32(rods[1].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_sinf4_u10(degrees_to_radians(v_xy_rotation_angle)))),
        vmulq_f32(vdupq_n_f32(rods[1].height), Sleef_sinf4_u10(degrees_to_radians(v_z_rotation_angle)))
    };
    if(DEBUG){
        float z_rotation_angle[4];
        vst1q_f32(z_rotation_angle, v_z_rotation_angle);
        std::cout << "second rod z_rotation_angle: " << z_rotation_angle[0] << std::endl;
        float cos_z_rotation_angle[4];
        vst1q_f32(cos_z_rotation_angle, Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)));
        std::cout << "second rod cos_z_rotation_angle: " << cos_z_rotation_angle[0] << std::endl;
        vst1q_f32(xy_rotation_angle, v_xy_rotation_angle);
        std::cout << "second rod xy_rotation_angle: " << xy_rotation_angle[0] << std::endl;

        float local_top_center[3][4];
        vst1q_f32(local_top_center[0], v_local_top_center[0]);
        vst1q_f32(local_top_center[1], v_local_top_center[1]);
        vst1q_f32(local_top_center[2], v_local_top_center[2]);
        std::cout << "second rod local_top_center: " << std::endl;
        std::cout << "(" << local_top_center[0][0] << ", " << local_top_center[1][0] << ", " << local_top_center[2][0] << ")" << std::endl;
    }
    v_top_center[0] = vaddq_f32(v_bottom_center[0], v_local_top_center[0]);
    v_top_center[1] = vaddq_f32(v_bottom_center[1], v_local_top_center[1]);
    v_top_center[2] = vaddq_f32(v_bottom_center[2], v_local_top_center[2]);

    if(DEBUG){
        vst1q_f32(top_center[0], v_top_center[0]);
        vst1q_f32(top_center[1], v_top_center[1]);
        vst1q_f32(top_center[2], v_top_center[2]);
        vst1q_f32(bottom_center[0], v_bottom_center[0]);
        vst1q_f32(bottom_center[1], v_bottom_center[1]);
        vst1q_f32(bottom_center[2], v_bottom_center[2]);
        posture[1].bottom_center.x = bottom_center[0][0];
        posture[1].bottom_center.y = bottom_center[1][0];
        posture[1].bottom_center.z = bottom_center[2][0];
        posture[1].top_center.x = top_center[0][0];
        posture[1].top_center.y = top_center[1][0];
        posture[1].top_center.z = top_center[2][0];
        posture[1].length = rods[1].length;
        posture[1].width = rods[1].width;
        posture[1].height = rods[1].height;
    }
    result.FK_time += get_elapsed_nanoseconds(FK_start_time);
    CC_start_time = std::chrono::steady_clock::now();
    result.valid = batchSphereCollisionDetection(v_top_center, v_bottom_center, 1, obstacles, DEBUG);
    result.CC_time += get_elapsed_nanoseconds(CC_start_time);
    if(result.valid == false){
        if(DEBUG){
            std::cout << "Collision detected for 2nd rod." << std::endl;
        }
        return result;
    }

    // third rod, x
    FK_start_time = std::chrono::steady_clock::now();
    v_bottom_center[0] = v_top_center[0];
    v_bottom_center[1] = v_top_center[1];
    v_bottom_center[2] = v_top_center[2];
    v_z_rotation_angle = vsubq_f32(v_z_rotation_angle, vsubq_f32(vdupq_n_f32(90.0f), v_angle2));
    v_local_top_center[0] = vmulq_f32(vdupq_n_f32(rods[2].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_cosf4_u10(degrees_to_radians(v_xy_rotation_angle))));
    v_local_top_center[1] = vmulq_f32(vdupq_n_f32(rods[2].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_sinf4_u10(degrees_to_radians(v_xy_rotation_angle))));
    v_local_top_center[2] = vmulq_f32(vdupq_n_f32(rods[2].height), Sleef_sinf4_u10(degrees_to_radians(v_z_rotation_angle)));
    v_top_center[0] = vaddq_f32(v_bottom_center[0], v_local_top_center[0]);
    v_top_center[1] = vaddq_f32(v_bottom_center[1], v_local_top_center[1]);
    v_top_center[2] = vaddq_f32(v_bottom_center[2], v_local_top_center[2]);

    if(DEBUG){
        vst1q_f32(top_center[0], v_top_center[0]);
        vst1q_f32(top_center[1], v_top_center[1]);
        vst1q_f32(top_center[2], v_top_center[2]);
        vst1q_f32(bottom_center[0], v_bottom_center[0]);
        vst1q_f32(bottom_center[1], v_bottom_center[1]);
        vst1q_f32(bottom_center[2], v_bottom_center[2]);
        posture[2].bottom_center.x = bottom_center[0][0];
        posture[2].bottom_center.y = bottom_center[1][0];
        posture[2].bottom_center.z = bottom_center[2][0];
        posture[2].top_center.x = top_center[0][0];
        posture[2].top_center.y = top_center[1][0];
        posture[2].top_center.z = top_center[2][0];
        posture[2].length = rods[2].length;
        posture[2].width = rods[2].width;
        posture[2].height = rods[2].height;
    }
    result.FK_time += get_elapsed_nanoseconds(FK_start_time);
    CC_start_time = std::chrono::steady_clock::now();
    result.valid = batchSphereCollisionDetection(v_top_center, v_bottom_center, 2, obstacles, DEBUG);
    result.CC_time += get_elapsed_nanoseconds(CC_start_time);
    if(result.valid == false){
        if(DEBUG){
            std::cout << "Collision detected for 3rd rod." << std::endl;
        }
        return result;
    }

    // fourth rod, -x
    FK_start_time = std::chrono::steady_clock::now();
    v_bottom_center[0] = v_top_center[0];
    v_bottom_center[1] = v_top_center[1];
    v_bottom_center[2] = v_top_center[2];
    v_z_rotation_angle = vsubq_f32(v_z_rotation_angle, vsubq_f32(v_angle3, vdupq_n_f32(90.0f)));
    v_local_top_center[0] = vmulq_f32(vdupq_n_f32(rods[3].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_cosf4_u10(degrees_to_radians(v_xy_rotation_angle))));
    v_local_top_center[1] = vmulq_f32(vdupq_n_f32(rods[3].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_sinf4_u10(degrees_to_radians(v_xy_rotation_angle))));
    v_local_top_center[2] = vmulq_f32(vdupq_n_f32(rods[3].height), Sleef_sinf4_u10(degrees_to_radians(v_z_rotation_angle)));
    v_top_center[0] = vaddq_f32(v_bottom_center[0], v_local_top_center[0]);
    v_top_center[1] = vaddq_f32(v_bottom_center[1], v_local_top_center[1]);
    v_top_center[2] = vaddq_f32(v_bottom_center[2], v_local_top_center[2]);

    if(DEBUG){
        vst1q_f32(top_center[0], v_top_center[0]);
        vst1q_f32(top_center[1], v_top_center[1]);
        vst1q_f32(top_center[2], v_top_center[2]);
        vst1q_f32(bottom_center[0], v_bottom_center[0]);
        vst1q_f32(bottom_center[1], v_bottom_center[1]);
        vst1q_f32(bottom_center[2], v_bottom_center[2]);
        posture[3].bottom_center.x = bottom_center[0][0];
        posture[3].bottom_center.y = bottom_center[1][0];
        posture[3].bottom_center.z = bottom_center[2][0];
        posture[3].top_center.x = top_center[0][0];
        posture[3].top_center.y = top_center[1][0];
        posture[3].top_center.z = top_center[2][0];
        posture[3].length = rods[3].length;
        posture[3].width = rods[3].width;
        posture[3].height = rods[3].height;
    }
    result.FK_time += get_elapsed_nanoseconds(FK_start_time);
    CC_start_time = std::chrono::steady_clock::now();
    result.valid = batchSphereCollisionDetection(v_top_center, v_bottom_center, 3, obstacles, DEBUG);
    result.CC_time += get_elapsed_nanoseconds(CC_start_time);
    if(result.valid == false){
        if(DEBUG){
            std::cout << "Collision detected for 4th rod." << std::endl;
        }
        return result;
    }

    // fifth rod, y
    FK_start_time = std::chrono::steady_clock::now();
    v_bottom_center[0] = v_top_center[0];
    v_bottom_center[1] = v_top_center[1];
    v_bottom_center[2] = v_top_center[2];
    v_local_top_center[0] = vmulq_f32(vdupq_n_f32(rods[4].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_cosf4_u10(degrees_to_radians(v_xy_rotation_angle))));
    v_local_top_center[1] = vmulq_f32(vdupq_n_f32(rods[4].height), vmulq_f32(Sleef_cosf4_u10(degrees_to_radians(v_z_rotation_angle)), Sleef_sinf4_u10(degrees_to_radians(v_xy_rotation_angle))));
    v_local_top_center[2] = vmulq_f32(vdupq_n_f32(rods[4].height), Sleef_sinf4_u10(degrees_to_radians(v_z_rotation_angle)));
    v_top_center[0] = vaddq_f32(v_bottom_center[0], v_local_top_center[0]);
    v_top_center[1] = vaddq_f32(v_bottom_center[1], v_local_top_center[1]);
    v_top_center[2] = vaddq_f32(v_bottom_center[2], v_local_top_center[2]);

    if(DEBUG){
        vst1q_f32(top_center[0], v_top_center[0]);
        vst1q_f32(top_center[1], v_top_center[1]);
        vst1q_f32(top_center[2], v_top_center[2]);
        vst1q_f32(bottom_center[0], v_bottom_center[0]);
        vst1q_f32(bottom_center[1], v_bottom_center[1]);
        vst1q_f32(bottom_center[2], v_bottom_center[2]);
        posture[4].bottom_center.x = bottom_center[0][0];
        posture[4].bottom_center.y = bottom_center[1][0];
        posture[4].bottom_center.z = bottom_center[2][0];
        posture[4].top_center.x = top_center[0][0];
        posture[4].top_center.y = top_center[1][0];
        posture[4].top_center.z = top_center[2][0];
        posture[4].length = rods[4].length;
        posture[4].width = rods[4].width;
        posture[4].height = rods[4].height;
    }
    result.FK_time += get_elapsed_nanoseconds(FK_start_time);
    CC_start_time = std::chrono::steady_clock::now();
    result.valid = batchSphereCollisionDetection(v_top_center, v_bottom_center, 4, obstacles, DEBUG);
    result.CC_time += get_elapsed_nanoseconds(CC_start_time);
    if(result.valid == false){
        if(DEBUG){
            std::cout << "Collision detected for 5th rod." << std::endl;
        }
        return result;
    }

    if(DEBUG){
        for(int i=0; i<5; i++){
            std::cout << "Rod " << i+1 << ": ";
            std::cout << "Bottom Center: " << posture[i].bottom_center << ", Top Center: " << posture[i].top_center;
            std::cout << ", Length: " << posture[i].length << ", Width: " << posture[i].width << ", Height: " << posture[i].height;
            std::cout << std::endl;
        }
    }

    return result; // No collision detected for all rods

}
