#pragma once
#include <vector>

// #define universal constants here, like
// #define SBPL_MAGIC_ARM_COST 40

namespace multi_agent_planner {
    const int ROBOT_DOF = 2;
    const int NUM_ROBOTS = 1;

    const double NOMINAL_Z = 0.2;

    // number of planner queues will be 1 (anchor - q_0) + NUM_LEADERS
    const int NUM_LEADERS = 1;  // MUST be <= NUM_ROBOTS

    // has to contain elements < NUM_ROBOTS and number of elements = NUM_LEADERS
    const std::vector<int> LEADER_IDS { 0 };  

    // just a handy shortcut
    const int PLANNING_DOF = ROBOT_DOF * NUM_ROBOTS;

    class RobotStateElement {
    public:
        enum {
            X,
            Y
            // THETA   // possibly. maybe later.
        };
    };
}