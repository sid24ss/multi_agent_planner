#include <multi_agent_planner/StateReps/DiscRobotState.h>
#include <multi_agent_planner/LoggerNames.h>

using namespace multi_agent_planner;
using namespace std;

bool DiscRobotState::operator==(const DiscRobotState& other) const {
    return (m_coords == other.m_coords);
}

bool DiscRobotState::operator!=(const DiscRobotState& other) const {
    return !(*this == other);
}

DiscRobotState::DiscRobotState(std::vector<int> coords) :
    m_coords(coords) { }


DiscRobotState::DiscRobotState(ContRobotState cont_r_state)
    : m_coords(ROBOT_DOF,0)
{
    int vx, vy, vz;
    m_occupancy_grid->worldToGrid(cont_r_state.x(), 
                                  cont_r_state.y(), 
                                  NOMINAL_Z,
                                  vx, vy, vz);

    m_coords[RobotStateElement::X] = vx;
    m_coords[RobotStateElement::Y] = vy;
}

int DiscRobotState::convertContDistance(double distance){
    double distance_res = m_occupancy_grid->getResolution();
    return static_cast<int>((distance + distance_res*0.5)/distance_res);
}


ContRobotState DiscRobotState::getContRobotState() const {
    return ContRobotState(*this);
}

void DiscRobotState::printToDebug(char* logger) {
    ROS_DEBUG_NAMED(logger, "%d %d",
                                m_coords[RobotStateElement::X],
                                m_coords[RobotStateElement::Y]);
}