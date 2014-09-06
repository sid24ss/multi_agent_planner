#pragma once
#include <vector>

// #define universal constants here, like
// #define SBPL_MAGIC_ARM_COST 40
#define PLANNING_DOF ROBOT_DOF*SwarmState::NUM_ROBOTS
#define NUM_LEADERS static_cast<int>(SwarmState::LEADER_IDS.size())

namespace multi_agent_planner {
    const int ROBOT_DOF = 2;
    // This is designed so that the swarm by itself can rotate, while the robots
    // only move in (x,y)
    const int SWARM_DOF = 2;

    const double NOMINAL_Z = 0.2;

    // number of planner queues will be 1 (anchor - q_0) + NUM_LEADERS
    // const int NUM_LEADERS = 4;  // MUST be <= NUM_ROBOTS

    // has to contain elements < NUM_ROBOTS and number of elements = NUM_LEADERS
    // const std::vector<int> LEADER_IDS { 0, 1, 2, 3 };  

    // just a handy shortcut
    // const int PLANNING_DOF = ROBOT_DOF * NUM_ROBOTS;

    class RobotStateElement {
    public:
        enum {
            X,
            Y
            // THETA   // possibly. maybe later.
        };
    };
    // TODO : SwarmStateElement
}