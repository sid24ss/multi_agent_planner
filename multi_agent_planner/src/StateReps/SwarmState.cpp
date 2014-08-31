#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/Constants.h>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

SwarmState::SwarmState(std::vector<RobotState> robots_pose) : m_robot_pose(robots_pose){ }

bool SwarmState::operator==(const SwarmState& other){
    // Note : Checks continuous equality
    return (m_robots_pose == other.m_robots_pose);
}

bool SwarmState::isDiscreteEqual(const SwarmState& first,
                                const SwarmState& second)
{
    // TODO : Make sure DiscRobotState takes in a vector of robot states and spits out a vector
    // of disc robot states
    return (DiscRobotState(first.robots_pose())
            == DiscRobotState(second.robots_pose()));
}

bool SwarmState::operator!=(const SwarmState& other){
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
