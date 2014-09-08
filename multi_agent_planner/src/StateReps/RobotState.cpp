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

RobotState::RobotState(std::vector<double> values)
{
    m_cont_robot_state = ContRobotState(values);
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

void RobotState::visualize(bool leader = false) const {
    Visualizer::visualizeRobot("planner", *this, leader);
}

std::vector<DiscRobotState> RobotState::getDiscStates(std::vector<RobotState> robot_states) {
    std::vector<DiscRobotState> disc_states;
    for (auto& state : robot_states) {
        disc_states.push_back(state.getDiscRobotState());
    }
    return disc_states;
}

bool RobotState::interpolate(const RobotState& start, const RobotState& end,
                            int num_interp_steps,
                            std::vector<RobotState>& interm_robot_steps)
{
    ContRobotState c_start = start.getContRobotState();
    ContRobotState c_end = end.getContRobotState();

    // interm_robot_steps will only give the intermediate steps. Add the source
    // and the end externally if need be
    double dx = (c_end.x() - c_start.x());
    double dy = (c_end.y() - c_start.y());
    double step_mult = 1.0f/static_cast<double>(num_interp_steps + 1);

    for (size_t i = 1; i <= static_cast<size_t>(num_interp_steps); i++) {
        ContRobotState c_robot_state;
        c_robot_state.x(c_start.x() + dx * static_cast<double>(i) * step_mult);
        c_robot_state.y(c_start.y() + dy * static_cast<double>(i) * step_mult);
        interm_robot_steps.push_back(RobotState(c_robot_state));
    }
    assert(num_interp_steps == static_cast<int>(interm_robot_steps.size()));
    return true;
}

void RobotState::vectorToRobotStates(std::vector<double> values,
                                    std::vector<RobotState>& robot_states)
{
    assert(static_cast<int>(values.size()) == PLANNING_DOF);
    auto begin = values.begin();
    for (int i = 0; i < SwarmState::NUM_ROBOTS; i++) {
        std::vector<double> this_robot_coords(begin, begin + ROBOT_DOF);
        robot_states.push_back(RobotState(this_robot_coords));
        begin = begin + ROBOT_DOF;
    }
}