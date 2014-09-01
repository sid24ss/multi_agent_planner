#include <multi_agent_planner/StateReps/RobotState.h>
#include <multi_agent_planner/LoggerNames.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Visualizer.h>
#include <vector>
#include <sstream>

using namespace multi_agent_planner;

bool RobotState::operator==(const RobotState& other) const {
    return m_cont_robot_state == other.getContRobotState();
}

bool RobotState::operator!=(const RobotState& other) const {
    return !(*this == other);
}

RobotState::RobotState(ContRobotState robot_state):
    m_cont_robot_state (robot_state) {}

RobotState::RobotState(DiscRobotState disc_robot_state) :
    m_cont_robot_state (disc_robot_state) {}

void RobotState::printToDebug(char* log_level) const {
    // will print the default (continuous) coordinates
    ROS_DEBUG_NAMED(log_level, "%f %f\n", 
                                m_cont_robot_state.x(),
                                m_cont_robot_state.y());
}

void RobotState::printToFile(FILE *& path) const {
    fprintf(path, "%f %f\n", 
                   m_cont_robot_state.x(),
                   m_cont_robot_state.y());
}

void RobotState::printToInfo(char* log_level) const {
    // will print the default (continuous) coordinates
    ROS_INFO_NAMED(log_level, "%f %f\n", 
                                m_cont_robot_state.x(),
                                m_cont_robot_state.y());
}

void RobotState::visualize(bool leader = false){
    // TODO
    // visualizer::visualizeFootprint(x, y) -> this should just take care of
    // visualizing the robot.
    // which means that the footprint must be configured somewher globally and
    // the visualizer should be aware of it.
}

std::vector<DiscRobotState> RobotState::getDiscStates(std::vector<RobotState> robot_states) {
    std::vector<DiscRobotState> disc_states;
    for (auto& state : robot_states) {
        disc_states.push_back(state.getDiscRobotState());
    }
    return disc_states;
}