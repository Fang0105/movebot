#include <arm_neon.h>
#include <iostream>
#include <vector>

struct Points {
    float x[4];
    float y[4];
    float z[4];
};

int main() {
    // Points A = {
    //     .x = {1, 2, 3, 4},
    //     .y = {5, 6, 7, 8},
    //     .z = {9, 10, 11, 12}
    // };

    // Points B = {
    //     .x = {10, 20, 30, 40},
    //     .y = {50, 60, 70, 80},
    //     .z = {90, 100, 110, 120}
    // };

    // Points C;

    // // 向量加總：C = A + B
    // float32x4_t ax = vld1q_f32(A.x);
    // float32x4_t ay = vld1q_f32(A.y);
    // float32x4_t az = vld1q_f32(A.z);

    // float32x4_t bx = vld1q_f32(B.x);
    // float32x4_t by = vld1q_f32(B.y);
    // float32x4_t bz = vld1q_f32(B.z);

    // float32x4_t cx = vaddq_f32(ax, bx);
    // float32x4_t cy = vaddq_f32(ay, by);
    // float32x4_t cz = vaddq_f32(az, bz);

    // vst1q_f32(C.x, cx);
    // vst1q_f32(C.y, cy);
    // vst1q_f32(C.z, cz);

    // for (int i = 0; i < 4; ++i) {
    //     std::cout << "C[" << i << "] = ("
    //               << C.x[i] << ", "
    //               << C.y[i] << ", "
    //               << C.z[i] << ")\n";
    // }
    float32x4_t v1 = vdupq_n_f32(1.0f);
    float32x4_t v2 = vdupq_n_f32(2.0f);

    float32x4_t v3 = vaddq_f32(v1, v2);
    float result[4];
    vst1q_f32(result, v3);
    for (int i = 0; i < 4; ++i) {
        std::cout << "result[" << i << "] = " << result[i] << std::endl;
    }

    std::vector<float> vec;

    vec.resize(4);
    vec[0] = 1.0f;
    vec[1] = 2.0f;
    vec[2] = 3.0f;
    vec[3] = 4.0f;
    for(float &v : vec) {
        std::cout << v << " ";
    }

    vec.resize(4);
    vec[0] = 2.0f;
    vec[1] = 4.0f;
    vec[2] = 6.0f;
    vec[3] = 8.0f;
    for(float &v : vec) {
        std::cout << v << " ";
    }

    return 0;
}
