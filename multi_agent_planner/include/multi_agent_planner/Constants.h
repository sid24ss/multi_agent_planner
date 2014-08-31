#pragma once
namespace multi_agent_planner {
    const int ROBOT_DOF = 2;
    const int NUM_ROBOTS = 1;

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