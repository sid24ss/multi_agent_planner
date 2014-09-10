#include <multi_agent_planner/StateReps/ContRobotState.h>
#include <multi_agent_planner/LoggerNames.h>

using namespace multi_agent_planner;
using namespace std;

bool ContRobotState::operator==(const ContRobotState& other) const {
    return (m_coords == other.m_coords);
}

bool ContRobotState::operator!=(const ContRobotState& other) const {
    return !(*this == other);
}

ContRobotState ContRobotState::operator+(const ContRobotState& other) const {
    ContRobotState new_state;
    new_state.x(x() + other.x());
    new_state.y(y() + other.y());
    return new_state;
}

ContRobotState::ContRobotState(const DiscRobotState& disc_r_state) :
    m_coords(ROBOT_DOF, 0) {
    double vz;
    m_occupancy_grid->gridToWorld(disc_r_state.x(),
                                  disc_r_state.y(),
                                  0,
                                  m_coords[RobotStateElement::X],
                                  m_coords[RobotStateElement::Y],
                                  vz);
}

DiscRobotState ContRobotState::getDiscRobotState() const
{
    return DiscRobotState(*this);
}

double ContRobotState::distance(const ContRobotState& start, const ContRobotState& end){
    double dX = end.x() - start.x();
    double dY = end.y() - start.y();
    return pow((pow(dX,2) + pow(dY,2)),.5);
}

void ContRobotState::printToDebug(char* logger){
    ROS_DEBUG_NAMED(logger, "%f %f", x(),y());
}