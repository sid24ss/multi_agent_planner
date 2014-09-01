#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

SwarmState::SwarmState(std::vector<RobotState> robots_pose)
        :   m_robots_pose(robots_pose){ }

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
    for (size_t i =0; i < m_robots_pose.size(); ++i) {
        ROS_DEBUG_NAMED(logger, "\trobot:%d", static_cast<int>(i));
        m_robots_pose[i].getDiscRobotState().printToDebug(logger);
    }
}

void SwarmState::printContToDebug(char* logger) const {
     for (size_t i =0; i < m_robots_pose.size(); ++i) {
        ROS_DEBUG_NAMED(logger, "\trobot:%d", static_cast<int>(i));
        m_robots_pose[i].getContRobotState().printToDebug(logger);
    }
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