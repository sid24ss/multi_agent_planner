#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <multi_agent_planner/Visualizer.h>
#include <sstream>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

SwarmState::SwarmState(std::vector<RobotState> robots_pose)
        :   m_robots_pose(robots_pose),
            m_leader(-1) { }

SwarmState::SwarmState(const SwarmState& other)
    :   m_robots_pose(other.m_robots_pose),
        m_leader(other.m_leader) { }            

bool SwarmState::operator==(const SwarmState& other) const {
    // Note : Checks continuous equality
    return (m_robots_pose == other.m_robots_pose);
}

bool SwarmState::isDiscreteEqual(const SwarmState& first,
                                const SwarmState& second)
{
    return (RobotState::getDiscStates(first.robots_pose())
            == RobotState::getDiscStates(second.robots_pose()));
}

bool SwarmState::operator!=(const SwarmState& other) const {
    return !(*this == other);
}

void SwarmState::printToDebug(char* logger) const {
    std::stringstream ss;
    for (size_t i =0; i < m_robots_pose.size(); ++i) {
        ss << " |" << i << "| ";
        ss << vectorToString(m_robots_pose[i].getDiscRobotState().coords());
    }
    ROS_DEBUG_NAMED(logger, "%s", ss.str().c_str());
}

void SwarmState::printContToDebug(char* logger) const {
    std::stringstream ss;
    for (size_t i =0; i < m_robots_pose.size(); ++i) {
        ss << " |" << i << "| ";
        ss << vectorToString(m_robots_pose[i].getContRobotState().coords());
    }
    ROS_DEBUG_NAMED(logger, "%s", ss.str().c_str());
}

void SwarmState::coords(std::vector<int> coords) {
    assert(static_cast<int>(coords.size()) == PLANNING_DOF);
    auto it = coords.begin();
    for (size_t i = 0; i < static_cast<size_t>(NUM_ROBOTS); i++) {
        DiscRobotState robot_state(std::vector<int> (it, it + ROBOT_DOF));
        m_robots_pose[i] = RobotState(robot_state);
    }
}

std::vector<int> SwarmState::coords() const {
    std::vector<int> coords;
    for (auto& robot_state : m_robots_pose) {
        combineVectors<int>(robot_state.getDiscRobotState().coords(), coords);
    }
    return coords;
}

bool SwarmState::interpolate(const SwarmState& start, const SwarmState& end,
        int num_interp_steps,
        std::vector<SwarmState>& interm_swarm_steps)
{
    std::vector< std::vector<RobotState> > interm_robot_states;
    interm_robot_states.resize(start.robots_pose().size());
    for (size_t i = 0; i < start.robots_pose().size(); i++) {
        RobotState::interpolate(start.robots_pose()[i],
                                end.robots_pose()[i],
                                num_interp_steps,
                                interm_robot_states[i]);
    }
    // sanity check
    assert(num_interp_steps == static_cast<int>(interm_robot_states[0].size()));
    // we have a vector that's NUM_ROBOTS size, each of which is a vector that's
    // num_interp_steps size
    // make swarm states out of them and push back to the list
    for (size_t i = 0; i < static_cast<size_t>(num_interp_steps); ++i) {
        // runs over the number of steps for each robot
        std::vector<RobotState> robots_list;
        for (size_t j = 0; j < interm_robot_states.size(); j++) {
            // runs over the list of robots

            // we want it in the order of the robots.
            robots_list.push_back(interm_robot_states[j][i]);
        }
        interm_swarm_steps.push_back(SwarmState(robots_list));
    }
    assert(static_cast<int>(interm_swarm_steps.size()) == num_interp_steps);
    return true;
}

void SwarmState::visualize() const {
    Visualizer::visualizeSwarm("swarm_state", *this);
}

