#include <iostream>
#include <fstream>
#include <random>
#include "arm.h"
#include "RRT.h"
#include "utils.h"
#include <algorithm>


RRT rrt;
Arm arm;
int random_seed;

Configuration sampleAConfiguration(Configuration &goal){
    // static std::random_device rd;
    static std::mt19937 gen(random_seed);

    std::uniform_int_distribution<int> toward_goal(0, 100);
    if(toward_goal(gen) <= 5){
        // std::cout << "====== Sample Goal =====" << std::endl;
        return goal;
    }

    Configuration q(arm.joint_number);
    for(int i=0; i<arm.joint_number; i++){
        std::uniform_int_distribution<int> dist(arm.joint_angles_range[i].first, arm.joint_angles_range[i].second);
        q.joint_angles[i] = dist(gen);
    }
    return q;
}

bool reach(Configuration &q, Configuration &goal, int &reach_threshold){
    for(int i=0; i<arm.joint_number; i++){
        if(std::abs(q.joint_angles[i] - goal.joint_angles[i]) > reach_threshold){
            return false;
        }
    }
    return true;
}

int main(int argc, char *argv[]) {
    std::string input_dir = "input";
    std::string output_dir = "output";
    std::string arm_description_file = input_dir + "/" + "arm_description.txt";
    std::string start_end_file = input_dir + "/" + "start_end.txt";
    std::string output_file = output_dir + "/" + "path.txt";
    std::string execution_time_file = output_dir + "/" + "execution_time.txt";
    std::string obstacles_file = input_dir + "/" + "obstacles.txt";
    int max_iterations = 1000;
    int reach_threshold = 5;
    if (argc > 1) {
        arm_description_file = argv[1];
    }
    if (argc > 2) {
        start_end_file = argv[2];
    }
    if (argc > 3) {
        output_file = argv[3];
    }
    if(argc > 4){
        obstacles_file = argv[4];   
    }
    if(argc > 5){
        execution_time_file = argv[5];   
    }
    if(argc > 6){
        max_iterations = std::stoi(argv[6]);
    }
    if(argc > 7){
        reach_threshold = std::stoi(argv[7]);
    }
    if(argc > 8){
        random_seed = std::stoi(argv[8]);
    } else {
        random_seed = 50;
    }

    // ---------------------- Read arm information ----------------------
    std::ifstream in;
    in.open(arm_description_file);
    if(in.fail()){
        std::cerr << "Error opening input file: " << arm_description_file << std::endl;
        return 1;
    }
    int joint_number, rod_number;
    in >> joint_number >> rod_number;
    // Arm arm(joint_number, rod_number);
    arm.initialize(joint_number, rod_number);
    for(int i=0; i < joint_number; i++){
        in >> arm.axis[i] >> arm.joint_angles_range[i].first >> arm.joint_angles_range[i].second;
    }
    for(int i=0; i < rod_number; i++){
        double length, width, height;
        in >> length >> width >> height;
        arm.rods[i] = Rectangle(length, width, height);
    }
    in.close();
    // ------------------------------------------------------------------

    arm.printInfo();

    // ---------------------- Read start and end configuration ----------------------
    in.open(start_end_file);
    if(in.fail()){
        std::cerr << "Error opening input file: " << start_end_file << std::endl;
        return 1;
    }

    Configuration start(joint_number);
    for(int i=0; i < joint_number; i++){
        in >> start.joint_angles[i];
    }
    rrt.tree.push_back(start);

    Configuration goal(joint_number);
    for(int i=0; i < joint_number; i++){
        in >> goal.joint_angles[i];
    }
    in.close();

    std::cout << RED << BOLD << "start from : " << NONE << start;
    std::cout << RED << BOLD << "end at : " << NONE << goal;
    // ------------------------------------------------------------------

    // ---------------------- Read obstacles ----------------------
    in.open(obstacles_file);
    if(in.fail()){
        std::cerr << "Error opening input file: " << obstacles_file << std::endl;
        return 1;
    }
    int obstacles_number;
    in >> obstacles_number;
    std::cout << "obstacles_number: " << obstacles_number << std::endl;
    std::vector<Sphere> obstacles(obstacles_number);
    for(int i=0; i < obstacles_number; i++){
        float radius, x, y, z;
        in >> radius >> x >> y >> z;
        obstacles[i] = Sphere(radius, Point(x, y, z));
        // std::cout << rect;
    }
    in.close();
    
    
    // ------------------------------------------------------------
    // std::cout << "++++++++++\n";
    // for(int i=0; i<obstacles.size(); i++){
    //     std::cout << obstacles[i];
    // }
    // std::cout << "++++++++++\n";

    // if(arm.collisionDetection(start, obstacles)){
    //     std::cout << RED << BOLD << "Start configuration is in collision!" << NONE << std::endl;
    // }else{
    //     std::cout << GREEN << BOLD << "Start configuration is valid!" << NONE << std::endl;
    // }

    // arm.printPosture(start);


    auto start_time = std::chrono::steady_clock::now();
    auto MV_total_time = 0.0, FK_total_time = 0.0, CC_total_time = 0.0;
    Configuration current = start;
    int steps = 0, count = 0;
    while(reach(current, goal, reach_threshold)==false && steps < max_iterations){
        // std::cout << "Iteration: " << steps << std::endl;
        Configuration qRandom = sampleAConfiguration(goal);
        Configuration qNearest = rrt.findNearest(qRandom);
        Configuration qNew(arm.joint_number);

        Configuration diff = qRandom - qNearest;

        int norm = 0;
        for(int i=0; i<arm.joint_number; i++){
            norm += std::abs(diff.joint_angles[i]);
        }

        for (int i = 0; i < arm.joint_number; i++) {
            double step = ((double)diff.joint_angles[i] / norm) * rrt.step_size;
            int moved = std::round(step);
            qNew.joint_angles[i] = qNearest.joint_angles[i] + moved;
        }

        // std::cout << "qNearest: " << qNearest;

        // if(arm.collisionDetection(qNew, obstacles)){
        //     // std::cout << "qNew: " << qNew << " -> Collision detected!" << std::endl;
        //     continue;
        // }
        auto MV_start_time = std::chrono::steady_clock::now();
        validation_result result;
        result = arm.motionValidation(qNearest, qNew, obstacles);
        count++;
        CC_total_time += result.CC_time;
        FK_total_time += result.FK_time;
        MV_total_time += get_elapsed_nanoseconds(MV_start_time);
        if(!result.valid){
            // std::cout << RED << "qNew: " << qNew << " -> Motion validation failed!" << NONE << std::endl;
            continue;
        }else{
            // std::cout << GREEN << "qNew: " << qNew << " -> Motion validation passed!" << NONE << std::endl;
        }

        if(rrt.isExist(qNew)){
            continue;
        }
        steps++;

        // std::cout << "norm: " << norm << std::endl;
        // std::cout << "qNearest: " << qNearest;
        // std::cout << "qRandom: " << qRandom;
        // std::cout << "diff: " << diff;
        // std::cout << "qNew: " << qNew;
        // std::cout << std::endl;
        // TODO: Collision check
        rrt.connect(qNearest, qNew);
        current = qNew;
    }
    auto total_time = get_elapsed_nanoseconds(start_time);

    std::vector<Configuration> path = rrt.getPath(start, current);
    std::ofstream out;
    out.open(output_file);
    if(out.fail()){
        std::cerr << "Error opening output file: " << output_file << std::endl;
        return 1;
    }
    for(int i=path.size()-1; i>=0; i--){
        out << path[i];
    }
    out.close();
    std::cout << PURPLE << "Path saved to: " << NONE << output_file << std::endl;
    if(reach(current, goal, reach_threshold)){
        std::cout << GREEN << BOLD << "Path found!" << NONE << std::endl;
    }else{
        std::cout << RED << BOLD << "Path not found!" << NONE << std::endl;
    }
    std::cout << "steps: " << steps << std::endl;
    std::cout << "tree size: " << rrt.tree.size() << std::endl;
    std::cout << "max_iterations: " << max_iterations << std::endl;
    std::cout << "reach_threshold: " << reach_threshold << std::endl;

    // execution time 
    out.open(execution_time_file);
    if(out.fail()){
        std::cerr << "Error opening output file: " << execution_time_file << std::endl;
        return 1;
    }

    auto MV_avg_time = MV_total_time / count;
    auto FK_avg_time = FK_total_time / count;
    auto CC_avg_time = CC_total_time / count;

    out << "Total time: " << total_time / 1000000.0 << " ms" << std::endl;
    out << "Motion validation avg time: " << MV_avg_time / 1000000.0 << " ms" << std::endl;
    out << "Forward kinematic avg time: " << FK_avg_time / 1000000.0 << " ms" << std::endl;
    out << "Collision check avg time: " << CC_avg_time / 1000000.0 << " ms" << std::endl;

    out << "MV ratio: " << (MV_total_time * 100.0 / total_time) << "%" << std::endl;
    out << "FK ratio: " << (FK_total_time * 100.0 / total_time) << "%" << std::endl;
    out << "CC ratio: " << (CC_total_time * 100.0 / total_time) << "%" << std::endl;
    out.close();
    std::cout << PURPLE << "Execution time saved to: " << NONE << execution_time_file << std::endl;

    std::cout << "--------------------- Time Statistics --------------------" << std::endl;
    std::cout << "Total time: " << total_time / 1000000.0 << " ms" << std::endl;
    std::cout << "Motion validation avg time: " << MV_avg_time / 1000000.0 << " ms" << std::endl;
    std::cout << "Forward kinematic avg time: " << FK_avg_time / 1000000.0 << " ms" << std::endl;
    std::cout << "Collision check avg time: " << CC_avg_time / 1000000.0 << " ms" << std::endl;

    std::cout << "MV ratio: " << (MV_total_time * 100.0 / total_time) << "%" << std::endl;
    std::cout << "FK ratio: " << (FK_total_time * 100.0 / total_time) << "%" << std::endl;
    std::cout << "CC ratio: " << (CC_total_time * 100.0 / total_time) << "%" << std::endl;

    // arm.calculatePosture(goal);
    // arm.printPosture(goal);
    

    // for(auto i:rrt.tree){
    //     std::cout << i;
    // }

    

    // arm.motionValidation(path[0], path[1], obstacles, true);
    
    // Configuration c(5, {0, 90, 0, 0, 90});
    // std::vector<Configuration> tem(4, c);
    // arm.batchCollisionDetection(tem, obstacles, true);

    return 0;
}
