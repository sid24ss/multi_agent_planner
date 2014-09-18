#include <multi_agent_planner/Visualizer.h>

using namespace multi_agent_planner;

std::unique_ptr<SwarmViz> Visualizer::swarmVizPtr;

void Visualizer::createSwarmVizInstance(ros::NodeHandle nh,
    std::string ref_frame) {
    swarmVizPtr.reset(new SwarmViz(nh));
    swarmVizPtr->setReferenceFrame(ref_frame);
}

void Visualizer::configureRobotParams(const RobotDescriptionParams& params) {
    swarmVizPtr->configureRobotParams(params.robot_radius, NOMINAL_Z);
}

void Visualizer::visualizeRobot(std::string ns,
    const RobotState& robot_state,
    bool leader = false)
{
    // visualize a circle (for now) at the x, y of the robot. 
    auto cont_robot_state = robot_state.getContRobotState();
    int hue = (leader) ? 0 : 120;
    swarmVizPtr->visualizeCircle(ns, cont_robot_state.x(),
                            cont_robot_state.y(), hue);
}

void Visualizer::visualizeSwarm(std::string ns,
                            const SwarmState& swarm_state)
{
    int leader_id = swarm_state.getLeader();
    std::vector<double> x;
    std::vector<double> y;
    std::vector<int> hues;
    for (size_t i = 0; i < swarm_state.robots_pose().size(); i++) {
        RobotState r_state = swarm_state.robots_pose()[i];
        x.push_back(r_state.getContRobotState().x());
        y.push_back(r_state.getContRobotState().y());
        hues.push_back((static_cast<int>(i)==leader_id)?0:120);
    }
    swarmVizPtr->visualizeQuadSwarm(ns, x, y, hues);
}