#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/Constants.h>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

GraphState::GraphState(SwarmState swarm_state) :
    m_swarm_state(swarm_state),
    m_coords(PLANNING_DOF, 0)
{
    updateStateFromSwarmState();
}

// GraphState::GraphState()

bool GraphState::operator==(const GraphState& other) const{
    return m_coords == other.m_coords;
}

bool GraphState::operator!=(const GraphState& other) const {
    return !(*this == other);
}

/*! \brief applies a generic mprim vector to this graph state.
 */
bool GraphState::applyMPrim(const GraphStateMotion& mprim){
    // the mprim is a vector of size ROBOT_DOF * NUM_ROBOTS
    return true;
}

void GraphState::printToDebug(char* logger) const {
    for (auto& c : m_coords) {
        ROS_DEBUG_NAMED(logger, "%d ", c);
    }
}

// void GraphState::printContToDebug(char* logger) const {

// }

void GraphState::updateStateFromSwarmState() {
    m_coords = m_swarm_state.coords();
}

void GraphState::updateSwarmStateFromGraphState() {
    m_swarm_state.coords(m_coords);
}