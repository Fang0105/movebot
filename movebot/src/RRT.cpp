#include <cmath>
#include "RRT.h"

Configuration RRT::findNearest(Configuration q){
    int nearest_index = 0;
    double min_norm_square = 0.0;

    for(int i=0;i<tree[0].joint_number;++i){
        min_norm_square += pow(RAD((double)tree[0].joint_angles[i]) - RAD((double)q.joint_angles[i]), 2);
    }

    int tree_size = tree.size();
    // std::cout << "tree_size: " << tree_size << std::endl;
    for(int i=1; i<tree_size; i++){
        double norm_square = 0.0;
        for(int j=0; j<tree[i].joint_number; j++){
            norm_square += pow(RAD((double)tree[i].joint_angles[j]) - RAD((double)q.joint_angles[j]), 2);
        }
        if(norm_square < min_norm_square){
            min_norm_square = norm_square;
            nearest_index = i;
        }
    }

    return tree[nearest_index];
}

void RRT::connect(Configuration parent_config, Configuration child_config){
    tree.push_back(child_config);
    parent[child_config] = parent_config;
}

std::vector<Configuration> RRT::getPath(Configuration start, Configuration goal){
    // std::cout << "getPath start: " << start;
    // std::cout << "getPath goal: " << goal;
    std::vector<Configuration> path;
    Configuration current = goal;
    while(current != start){
        path.push_back(current);
        current = parent[current];
    }
    path.push_back(start);
    return path;
}

bool RRT::isExist(Configuration q){
    int tree_size = tree.size();
    for(int i=0; i<tree_size; i++){
        if(tree[i] == q){
            return true;
        }
    }
    return false;
}