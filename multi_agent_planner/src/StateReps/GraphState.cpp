#include <multi_agent_planner/StateReps/GraphState.h>
#include <multi_agent_planner/Constants.h>
#include <multi_agent_planner/Utilities.h>
#include <sstream>
#include <boost/scoped_ptr.hpp>

using namespace multi_agent_planner;
using namespace boost;

GraphState::GraphState(SwarmState swarm_state) :
    m_swarm_state(swarm_state),
    m_coords(PLANNING_DOF+1, 0)
{
    updateStateFromSwarmState();
}

GraphState::GraphState(const GraphState& other)
    :   m_id(other.m_id),
        m_swarm_state(other.m_swarm_state),
        m_coords(other.m_coords)
{}

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
    // TODO
    // this should not be called just yet.
    assert(false);
    updateSwarmStateFromGraphState();
    return true;
}

void GraphState::printToDebug(char* logger) const {
    std::stringstream ss;
    ss << vectorToString(m_coords);
    ss << "\t leader : " << m_swarm_state.getLeader();
    ROS_DEBUG_NAMED(logger, "\t%s", ss.str().c_str());
}

void GraphState::swarm_state(SwarmState swarm_state) {
    m_swarm_state = swarm_state;
    updateStateFromSwarmState();
};

// void GraphState::printContToDebug(char* logger) const {
// }

void GraphState::updateStateFromSwarmState() {
    m_coords = m_swarm_state.coords();
    m_coords.push_back(m_swarm_state.getLeader());
}

void GraphState::updateSwarmStateFromGraphState() {
    std::vector<int> swarm_coords(m_coords.begin(), m_coords.end()-1);
    m_swarm_state.coords(swarm_coords);
    m_swarm_state.setLeader(m_coords.back());
    // m_swarm_state.coords(m_coords);
}

void GraphState::setLeader(int l) {
    m_coords.back() = l;
    m_swarm_state.setLeader(l);
}

std::vector<int> GraphState::getCoords() const{
    return m_coords;
}

int GraphState::getLeader() const {
    assert(m_coords.back() == m_swarm_state.getLeader());
    return m_swarm_state.getLeader();
}