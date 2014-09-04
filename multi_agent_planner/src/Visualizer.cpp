#include <multi_agent_planner/Visualizer.h>

using namespace multi_agent_planner;

std::unique_ptr<SwarmViz> swarmVizPtr;

void Visualizer::createSwarmVizInstance(ros::NodeHandle nh,
    std::string ref_frame) {
    swarmVizPtr.reset(new SwarmViz(nh));
    swarmVizPtr->setReferenceFrame(ref_frame);
}

void Visualizer::configureRobotParams(const RobotDescriptionParams& params) {
    swarmVizPtr->configureRobotParams(params.robot_radius, NOMINAL_Z);
}

void visualizeRobot(std::string ns,
    const RobotState& robot_state,
    bool leader = false)
{
    // visualize a circle (for now) at the x, y of the robot. 
    auto cont_robot_state = robot_state.getContRobotState();
    int hue = (leader) ? 0 : 120;
    swarmVizPtr->visualizeCircle(ns, cont_robot_state.x(),
                            cont_robot_state.y(), hue);
}