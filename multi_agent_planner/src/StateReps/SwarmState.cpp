#include <multi_agent_planner/StateReps/SwarmState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <multi_agent_planner/Visualizer.h>
#include <sstream>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

SwarmState::SwarmState(): m_robots_pose(SwarmState::NUM_ROBOTS, RobotState()) { }

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

void SwarmState::configureSwarmState(const SwarmDescriptionParams& params) {
    SwarmState::NUM_ROBOTS = static_cast<int>(
                static_cast<int>(params.relative_positions.size())/ROBOT_DOF);
    SwarmState::LEADER_IDS = params.leader_ids;
    // configure the relative positions
    // [from][to] -> robot_from - robot_to
    SwarmState::REL_POSITIONS.resize(SwarmState::NUM_ROBOTS);
    for (size_t i = 0; i < static_cast<size_t>(NUM_ROBOTS); i++) {
        auto& robot_rel_position = SwarmState::REL_POSITIONS[i];
        robot_rel_position.resize(SwarmState::NUM_ROBOTS);
        // this is the from robot. let's loop over to.
        for (size_t j = 0; j < static_cast<size_t>(NUM_ROBOTS); j++) {
            ContRobotState c_robot_state;
            c_robot_state.x(params.relative_positions[2*i] - params.relative_positions
                [2*j]);
            c_robot_state.y(params.relative_positions[2*i+1] - params.relative_positions
                [2*j+1]);
            robot_rel_position[j] = c_robot_state;
        }
    }
}

/**
 * @brief transform a SwarmState to the position described
 * @details The transform is computed from the robot index specified in the
 * pivot param
 * 
 * @param position A valid RobotStateElement vector
 * @param pivot The index of the robot that is at the location specified
 * by position; defaults to the 0th robot
 * @return The transformed SwarmState
 */
SwarmState SwarmState::transformSwarmToPos(std::vector<double> position,
                                            int pivot)
{
    assert(static_cast<int>(position.size()) == SWARM_DOF);
    std::vector<RobotState> robots_list(SwarmState::NUM_ROBOTS);
    for (size_t i = 0; i < static_cast<size_t>(SwarmState::NUM_ROBOTS); i++) {
        ContRobotState state;
        state.x(position[RobotStateElement::X] + SwarmState::REL_POSITIONS[i][
            pivot].x());
        state.y(position[RobotStateElement::Y] + SwarmState::REL_POSITIONS[i][
            pivot].y());
        robots_list[i] = RobotState(state);
    }
    return SwarmState(robots_list);
}