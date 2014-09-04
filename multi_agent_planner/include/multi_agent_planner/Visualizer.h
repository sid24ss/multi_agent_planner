#pragma once
#include <string>
#include <memory>
#include <ros/ros.h>
#include <multi_agent_planner/ParameterCatalog.h>
#include <multi_agent_planner/swarmViz.h>
#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/Constants.h>

namespace multi_agent_planner {
    /*! \brief Simple visualizer for swarms
     */
    class Visualizer {
        public:
            static void createSwarmVizInstance(ros::NodeHandle nh, std::string ref_frame);
            static void visualizeSwarm(std::string ns,
                                        const SwarmState& swarm_state);
            static void visualizeRobot(std::string ns,
                                        const RobotState& robot_state,
                                        bool leader);
            static void configureRobotParams(const RobotDescriptionParams&
                params);
            static std::unique_ptr<SwarmViz> swarmVizPtr;
    };
}
