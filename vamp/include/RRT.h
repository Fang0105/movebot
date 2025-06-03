#ifndef __RRT_H__
#define __RRT_H__

#include <vector>
#include <map>
#include "utils.h"



class RRT{
    public:
        std::vector<Configuration> tree;
        std::map<Configuration, Configuration> parent;
        int step_size = 10; // each angle can move 3 degrees at most
        // double step_size = 2.0; // each angle can move 3 degrees at most


        Configuration findNearest(Configuration q);
        void connect(Configuration parent_config, Configuration child_config);

        std::vector<Configuration> getPath(Configuration start, Configuration goal);

        bool isExist(Configuration q);
};


#endif